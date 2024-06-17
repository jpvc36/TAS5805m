#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#define FILENAME		"ppc3_output.h"
#define OUTPUT_NAME		"ppc3_output.bin"
#define MAX_NUMBERS		8*1024
#define CFG_META_DELAY		254
#define CFG_META_BURST		253
#define CFG_ASCII_TEXT		240
//#define DSP_BOOK_ONLY		0xaa
#define DSP_BOOK_ONLY		0x8c

char* read_file(const char* filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Unable to open file");
        exit(EXIT_FAILURE);
    }

    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);

    char *buffer = malloc(length + 1);
    if (!buffer) {
        perror("Unable to allocate buffer");
        exit(EXIT_FAILURE);
    }

    fread(buffer, 1, length, file);
    buffer[length] = '\0';

    fclose(file);
    return buffer;
}

void parse_hex_numbers(const char* buffer, unsigned char* numbers, int* count) {
    const char *ptr = buffer;                                                   // consecutive characters
    int num;
    *count = 0;

    while (*ptr) {
        if (sscanf(ptr, "{ 0x%x", &num) == 1 || sscanf(ptr, ", 0x%x", &num) == 1) {
            numbers[*count] = num;
            (*count)++;
        }
        if (sscanf(ptr, "{ CFG_META_BURST, %d }", &num) == 1) {
            numbers[*count] = CFG_META_BURST;
            (*count)++;
            numbers[*count] = num;
            (*count)++;
        }
        if (sscanf(ptr, "{ CFG_META_DELAY, %d },", &num) == 1) {
            numbers[*count] = CFG_META_DELAY;
            (*count)++;
            numbers[*count] = num;
            (*count)++;
        }
/*
		case CFG_ASCII_TEXT:				// skip n = s[i + 1] - 1 ascii characters
			i +=  s[i + 1];
			break;
*/
        ptr++;
    }
        printf("\n");
}

int main() {
    char *filename = FILENAME;
    char *outputname = OUTPUT_NAME;
    char *buffer = read_file(filename);
    char *output[25];
    unsigned char data, numbers[MAX_NUMBERS];
    unsigned int count = 0, book_page_reg_burst= 0;
    FILE* fptr;
    fptr = fopen(OUTPUT_NAME, "wb");
    if (fptr == NULL) {
        printf("The file is not opened. The program will "
               "now exit.");
        exit(0);
    }

    parse_hex_numbers(buffer, numbers, &count);
//  printf("%s",buffer);
//    printf("Read %d hex numbers:\n", count);

    numbers[0] = 0;
    int i = 0;
    for (i = 0; i < count; i++) {
//    if ((numbers[i] == CFG_META_BURST) && ((book_page_reg_burst & 0x000000ff) > 0) && (i%2 == 0))
    if ((book_page_reg_burst & 0x000000ff) > 0)
        book_page_reg_burst -= 0x01;
    if ((numbers[i] == CFG_META_BURST) && ((book_page_reg_burst & 0x000000ff) == 0) && (i%2 == 0))
        book_page_reg_burst = book_page_reg_burst & 0xffffff00 | numbers[i + 1]; 
    if ((numbers[i] == 0x00) && (numbers[i + 1] == 0x00) &&
       (numbers[i + 2] == 0x7f) && (numbers[i + 3] == 0x00) && (i%2 == 0))
            book_page_reg_burst = book_page_reg_burst & 0x00ffffff | 0x00000000; // 
    #ifdef DSP_BOOK_ONLY
    if (((book_page_reg_burst & 0x00ff00ff) == 0) && (i%2 == 0) &&
       (numbers[i+2] == 0x7f) && (numbers[i+3] == DSP_BOOK_ONLY))
            book_page_reg_burst = book_page_reg_burst & 0x00ffffff | DSP_BOOK_ONLY << 24; 
    #endif
    if (((book_page_reg_burst & 0x00ffffff) == 0) && (i%2 == 0) && (numbers[i] == 0x7f))
            book_page_reg_burst = book_page_reg_burst & 0x00ffffff | numbers[i + 1]  << 24; 
    #ifndef DSP_BOOK_ONLY
    if ((numbers[i] == 0x00) && ((book_page_reg_burst & 0x000000ff) == 0) && (i%2 == 0))
            book_page_reg_burst = (book_page_reg_burst & 0xff00ffff) | (numbers[i + 1] << 16); 
    #endif
    if (((book_page_reg_burst & 0x000000ff) == 0) && (i%2 == 0))
            book_page_reg_burst = (book_page_reg_burst & 0xffff00ff) | (numbers[i] << 8); 
//    printf("%X ", numbers[i]);
    #ifdef DSP_BOOK_ONLY
        if ((book_page_reg_burst & 0xff000000) == DSP_BOOK_ONLY << 24) {
            fputc(numbers[i],fptr);
            printf("0x%03x %x Book, Page, Reg 0x%08x Data 0x%02x\n", i, i%2, book_page_reg_burst, numbers[i]);
        }
    #endif
    #ifndef DSP_BOOK_ONLY
        fputc(numbers[i],fptr);
        if (i % 2) printf("0x%03x Book, Page, Reg 0x%08x Address 0x%02x Data 0x%02x\n", i / 2, book_page_reg_burst, numbers[i - 1], numbers[i]);
    #endif
    }
    #ifdef DSP_BOOK_ONLY
    fputc(0x00,fptr);
    fputc(0x00,fptr);
    fputc(0x7f,fptr);
    fputc(0x00,fptr);
    #endif
    fclose(fptr);
//    printf("Counter = 0x%x:\n", book_page_reg_burst);
        if (sscanf(filename, "%13s%", output) >= 1)
    printf("%s, %d registers read\n",output, count);
    printf("Output: %s\n", outputname);
//    printf("\n");

    free(buffer);
    return 0;
}
