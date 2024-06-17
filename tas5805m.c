// #define DEBUG
// SPDX-License-Identifier: GPL-2.0
//
// Driver for the TAS5805M Audio Amplifier
//
// Author: Andy Liu <andy-liu@ti.com>
// Author: Daniel Beer <daniel.beer@igorinstitute.com>
// Author: J.P. van Coolwijk <jpvc36@gmail.com>
//
// This is based on a driver originally written by Andy Liu at TI and
// posted here:
//
//    https://e2e.ti.com/support/audio-group/audio/f/audio-forum/722027/linux-tas5825m-linux-drivers
//
// It has been simplified a little and reworked for the 5.x ALSA SoC API.
// This driver works with a binary firmware file. When missing or invalid a minimal config for PVDD=24V is used.

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

/* Datasheet-defined registers on page 0, book 0 */
#define REG_PAGE		0x00
#define REG_RESET_CTRL		0x01
#define REG_DEVICE_CTRL_1	0x02
#define REG_DEVICE_CTRL_2	0x03
#define REG_SIG_CH_CTRL		0x28
#define REG_SAP_CTRL_1		0x33
#define REG_FS_MON		0x37
#define REG_BCK_MON		0x38
#define REG_CLKDET_STATUS	0x39
#define REG_VOL_CTL		0x4c
#define REG_AGAIN		0x54
#define REG_ADR_PIN_CTRL	0x60
#define REG_ADR_PIN_CONFIG	0x61
#define REG_CHAN_FAULT		0x70
#define REG_GLOBAL_FAULT1	0x71
#define REG_GLOBAL_FAULT2	0x72
#define REG_FAULT		0x78
#define REG_BOOK		0x7f

/* DEVICE_CTRL_2 register values */
#define DCTRL2_MODE_DEEP_SLEEP	0x00
#define DCTRL2_MODE_SLEEP	0x01
#define DCTRL2_MODE_HIZ		0x02
#define DCTRL2_MODE_PLAY	0x03

#define DCTRL2_MUTE		0x08
#define DCTRL2_DIS_DSP		0x10

/* REG_FAULT register values */
#define ANALOG_FAULT_CLEAR	0x80

/* PPC3 commands */
#define CFG_META_DELAY		0xfe
#define CFG_META_BURST		0xfd
#define CFG_ASCII_TEXT		0xf0

#define TAS5805M_RATES		(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
				SNDRV_PCM_RATE_192000)

#define TAS5805M_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
				SNDRV_PCM_FMTBIT_S24_3LE |\
				SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* This sequence of register writes also takes care of the
 * 5ms delay while we wait for the DSP to boot.
 */
static const uint8_t firmware_missing[] = {
	REG_PAGE,		0x00,
	REG_BOOK,		0x00,
	REG_DEVICE_CTRL_2,	0x02,
	REG_RESET_CTRL,		0x10,
	CFG_META_DELAY,		0x05,
	REG_AGAIN,		0x03,
	REG_FAULT,		0x80,
};

struct tas5805m_priv {
	struct i2c_client		*i2c;
	struct regulator		*pvdd;
	struct gpio_desc		*gpio_pdn_n;

	uint8_t				*dsp_cfg_data[3];
	int				dsp_cfg_len[3];

	struct regmap			*regmap;

	bool				is_powered;
	bool				is_muted;
	uint8_t				firmware_valid;
	u32				rate;

	struct work_struct		work;
	struct mutex			lock;
};

static void tas5805m_refresh(struct tas5805m_priv *tas5805m)
{
	struct regmap *rm = tas5805m->regmap;

	dev_dbg(&tas5805m->i2c->dev, "refresh: is_muted=%d, is_powered=%d\n",
		tas5805m->is_muted, tas5805m->is_powered);

	/* Set/clear digital soft-mute */
	regmap_write(rm, REG_DEVICE_CTRL_2,
		(tas5805m->is_muted ? DCTRL2_MUTE : 0) |
		DCTRL2_MODE_PLAY);
}

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(tas5805m_vol_tlv, -10350, 50, 1);
static const struct snd_kcontrol_new tas5805m_snd_controls[] = {
	SOC_SINGLE_TLV	("Master Playback Volume", REG_VOL_CTL, 0, 255, 1, tas5805m_vol_tlv),
};

/* Delay while we wait for the DSP to boot,
 * bursts of data and explanatory text in the firmware.
 */
static void send_cfg(struct regmap *rm,
		const uint8_t *s, unsigned int len)
{
	unsigned int i = 0;
	while (i < len) {
		switch (s[i]) {
		case CFG_META_DELAY:
			usleep_range((1000 * s[i + 1]), (1000 * s[i + 1]) + 10000);
			i++;
			break;
		case CFG_META_BURST:
			regmap_bulk_write(rm, s[i + 2], &s[i + 3], s[i + 1] - 1);
			i +=  s[i + 1];
			break;
		case CFG_ASCII_TEXT:				// skip n = s[i + 1] - 1 ascii characters
			i +=  s[i + 1];
			break;
		default:
			regmap_write(rm, s[i], s[i + 1]);
			i++;
			break;
		}
		i++;
	}
}

static void send_minimal_cfg(struct regmap *rm)
{
	send_cfg(rm, firmware_missing, ARRAY_SIZE(firmware_missing));
}

/* The TAS5805M DSP can't be configured until the I2S clock has been
 * present and stable for 5ms, or else it won't boot and we get no
 * sound.
 */
static int tas5805m_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(component->dev, "clock start\n");
		schedule_work(&tas5805m->work);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void do_work(struct work_struct *work)
{
	struct tas5805m_priv *tas5805m =
	       container_of(work, struct tas5805m_priv, work);
	struct regmap *rm = tas5805m->regmap;

	dev_dbg(&tas5805m->i2c->dev, "DSP startup\n");

	mutex_lock(&tas5805m->lock);
	/* We mustn't issue any I2C transactions until the I2S
	 * clock is stable. Furthermore, we must allow a 5ms
	 * delay after the first set of register writes to
	 * allow the DSP to boot before configuring it.
	 */
	usleep_range(5000, 10000);
	dev_dbg(&tas5805m->i2c->dev, "firmware_valid=%d\n",
			tas5805m->firmware_valid);
	switch (tas5805m->rate) {

		case 44100:
		case 88200:
			if (tas5805m->firmware_valid && 1<<0) {
				dev_dbg(&tas5805m->i2c->dev,
					"sample rate = %d Hz, firmware for DSP 88.2 kHz\n", tas5805m->rate);
				send_cfg(rm, tas5805m->dsp_cfg_data[0],
					tas5805m->dsp_cfg_len[0]);
			} else {
				dev_dbg(&tas5805m->i2c->dev,
					"sample rate = %d Hz, no firmware loaded\n", tas5805m->rate);
				send_minimal_cfg(rm);	
			}
			break;

		case 32000:
		case 48000:
		case 96000:
			if (tas5805m->firmware_valid && 1<<1) {
				dev_dbg(&tas5805m->i2c->dev,
					"sample rate = %d Hz, firmware for DSP 96 kHz\n", tas5805m->rate);
			send_cfg(rm, tas5805m->dsp_cfg_data[1], tas5805m->dsp_cfg_len[1]);
			} else {
				dev_dbg(&tas5805m->i2c->dev,
					"sample rate = %d Hz, no firmware loaded\n", tas5805m->rate);
				send_minimal_cfg(rm);	
			}
			break;

		case 176400:
		case 192000:
			if (tas5805m->firmware_valid && 1<<2) {
				dev_err(&tas5805m->i2c->dev,
					"sample rate = %d Hz, firmware for 176.4 / 192 kHz without DSP\n", tas5805m->rate);
			send_cfg(rm, tas5805m->dsp_cfg_data[2], tas5805m->dsp_cfg_len[2]);
			} else {
				dev_dbg(&tas5805m->i2c->dev,
					"sample rate = %d Hz, no firmware loaded\n", tas5805m->rate);
				send_cfg(rm, firmware_missing, ARRAY_SIZE(firmware_missing));
			}
			break;
	}
	tas5805m->is_powered = true;
	tas5805m_refresh(tas5805m);
	mutex_unlock(&tas5805m->lock);
}

static int tas5805m_dac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);
	struct regmap *rm = tas5805m->regmap;

	if (event & SND_SOC_DAPM_PRE_PMD) {
		unsigned int chan, global1, global2;

		dev_dbg(component->dev, "DSP shutdown\n");
		cancel_work_sync(&tas5805m->work);

		mutex_lock(&tas5805m->lock);
		if (tas5805m->is_powered) {
			tas5805m->is_powered = false;

			regmap_write(rm, REG_PAGE, 0x00);
			regmap_write(rm, REG_BOOK, 0x00);

			regmap_read(rm, REG_CHAN_FAULT, &chan);
			regmap_read(rm, REG_GLOBAL_FAULT1, &global1);
			regmap_read(rm, REG_GLOBAL_FAULT2, &global2);

			dev_dbg(component->dev, "fault regs: CHAN=%02x, "
				"GLOBAL1=%02x, GLOBAL2=%02x\n",
				chan, global1, global2);

			regmap_write(rm, REG_DEVICE_CTRL_2, DCTRL2_MODE_HIZ);
		}
		mutex_unlock(&tas5805m->lock);
	}

	return 0;
}

static const struct snd_soc_dapm_route tas5805m_audio_map[] = {
	{ "DAC", NULL, "DAC IN" },
	{ "OUT", NULL, "DAC" },
};

static const struct snd_soc_dapm_widget tas5805m_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0,
		tas5805m_dac_event, SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_component_driver soc_codec_dev_tas5805m = {
	.controls		= tas5805m_snd_controls,
	.num_controls		= ARRAY_SIZE(tas5805m_snd_controls),
	.dapm_widgets		= tas5805m_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas5805m_dapm_widgets),
	.dapm_routes		= tas5805m_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas5805m_audio_map),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int tas5805m_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);
	tas5805m->rate = params_rate(params);
	return 0;
}

static int tas5805m_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&tas5805m->lock);
	dev_dbg(component->dev, "set mute=%d (is_powered=%d)\n",
		mute, tas5805m->is_powered);

	tas5805m->is_muted = mute;
	if (tas5805m->is_powered)
		tas5805m_refresh(tas5805m);
	mutex_unlock(&tas5805m->lock);

	return 0;
}

static const struct snd_soc_dai_ops tas5805m_dai_ops = {
	.trigger		= tas5805m_trigger,
	.hw_params		= tas5805m_hw_params,
	.mute_stream		= tas5805m_mute,
	.no_capture_mute	= 1,
};

static struct snd_soc_dai_driver tas5805m_dai = {
	.name		= "tas5805m-amplifier",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= TAS5805M_RATES,
		.formats	= TAS5805M_FORMATS,
	},
	.ops		= &tas5805m_dai_ops,
};

static const struct regmap_config tas5805m_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	/* We have quite a lot of multi-level bank switching and a
	 * relatively small number of register writes between bank
	 * switches.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int tas5805m_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;
	struct tas5805m_priv *tas5805m;
	char filename[128];
	const char *config_name;
	const char *config_rate[3] = {"88.2kHz", "96kHz", "192kHz"};
	const struct firmware *fw[2];
	int ret;
	int i;

	regmap = devm_regmap_init_i2c(i2c, &tas5805m_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "unable to allocate register map: %d\n", ret);
		return ret;
	}

	tas5805m = devm_kzalloc(dev, sizeof(struct tas5805m_priv), GFP_KERNEL);
	if (!tas5805m)
		return -ENOMEM;

	tas5805m->i2c = i2c;
	tas5805m->pvdd = devm_regulator_get(dev, "pvdd");
	if (IS_ERR(tas5805m->pvdd)) {
		dev_err(dev, "failed to get pvdd supply: %ld\n",
			PTR_ERR(tas5805m->pvdd));
		return PTR_ERR(tas5805m->pvdd);
	}

	dev_set_drvdata(dev, tas5805m);
	tas5805m->regmap = regmap;
	tas5805m->gpio_pdn_n = devm_gpiod_get(dev, "pdn", GPIOD_OUT_LOW);
	if (IS_ERR(tas5805m->gpio_pdn_n)) {
		dev_err(dev, "error requesting PDN gpio: %ld\n",
			PTR_ERR(tas5805m->gpio_pdn_n));
		return PTR_ERR(tas5805m->gpio_pdn_n);
	}

	/* This configuration must be generated by PPC3. The file loaded
	 * consists of a sequence of register writes, where bytes at
	 * even indices are register addresses and those at odd indices
	 * are register values. See PPC3 output for more options.
	 */
	tas5805m->firmware_valid = 0x00;
	if (device_property_read_string(dev, "ti,dsp-config-name",
				&config_name))
		config_name = "default";

	for (i = 0; i < 3 ; i++) {
		snprintf(filename, sizeof(filename), "tas5805m_dsp_%s_%s.bin",
			config_name, config_rate[i]);
		ret = request_firmware_direct(&fw[i], filename, dev);

		if (!ret) {		
		tas5805m->dsp_cfg_len[i] = fw[i]->size;
		tas5805m->dsp_cfg_data[i] = devm_kmemdup(dev, fw[i]->data, fw[i]->size, GFP_KERNEL);
			if (!tas5805m->dsp_cfg_data[i]) {
				dev_err(dev, "firmware is not loaded, using minimal %s config for PVDD=24V\n", config_rate[i]);
			}
			if ((fw[i]->size < 2) || (fw[i]->size & 1)) {
				dev_err(dev, "firmware is invalid, using minimal %s config for PVDD=24V\n", config_rate[i]);
			} else {
			tas5805m->firmware_valid += 1<<i;
			}
		release_firmware(fw[i]);
		} else {
			dev_err(dev, "firmware not found, using minimal %s config for PVDD=24V\n", config_rate[i]);
		}
	}

	/* Do the first part of the power-on here, while we can expect
	 * the I2S interface to be quiet. We must raise PDN# and then
	 * wait 5ms before any I2S clock is sent, or else the internal
	 * regulator apparently won't come on.
	 *
	 * Also, we must keep the device in power down for 100ms or so
	 * after PVDD is applied, or else the ADR pin is sampled
	 * incorrectly and the device comes up with an unpredictable I2C
	 * address.
	 */

	ret = regulator_enable(tas5805m->pvdd);
	if (ret < 0) {
		dev_err(dev, "failed to enable pvdd: %d\n", ret);
		return ret;
	}

	usleep_range(100000, 150000);
	gpiod_set_value(tas5805m->gpio_pdn_n, 1);
	usleep_range(10000, 15000);

	INIT_WORK(&tas5805m->work, do_work);
	mutex_init(&tas5805m->lock);

	/* Don't register through devm. We need to be able to unregister
	 * the component prior to deasserting PDN#
	 */
	ret = snd_soc_register_component(dev, &soc_codec_dev_tas5805m,
					 &tas5805m_dai, 1);
	if (ret < 0) {
		dev_err(dev, "unable to register codec: %d\n", ret);
		gpiod_set_value(tas5805m->gpio_pdn_n, 0);
		regulator_disable(tas5805m->pvdd);
		return ret;
	}

	return 0;
}

static void tas5805m_i2c_remove(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct tas5805m_priv *tas5805m = dev_get_drvdata(dev);

	cancel_work_sync(&tas5805m->work);
	snd_soc_unregister_component(dev);
	gpiod_set_value(tas5805m->gpio_pdn_n, 0);
	usleep_range(10000, 15000);
	regulator_disable(tas5805m->pvdd);
}

static const struct i2c_device_id tas5805m_i2c_id[] = {
	{ "tas5805m", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5805m_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tas5805m_of_match[] = {
	{ .compatible = "ti,tas5805m", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5805m_of_match);
#endif

static struct i2c_driver tas5805m_i2c_driver = {
	.probe_new	= tas5805m_i2c_probe,
	.remove		= tas5805m_i2c_remove,
	.id_table	= tas5805m_i2c_id,
	.driver		= {
		.name		= "tas5805m",
		.of_match_table = of_match_ptr(tas5805m_of_match),
	},
};

module_i2c_driver(tas5805m_i2c_driver);

MODULE_AUTHOR("Andy Liu <andy-liu@ti.com>");
MODULE_AUTHOR("Daniel Beer <daniel.beer@igorinstitute.com>");
MODULE_DESCRIPTION("TAS5805M Audio Amplifier Driver");
MODULE_LICENSE("GPL v2");