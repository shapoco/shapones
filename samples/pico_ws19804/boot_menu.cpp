#include "boot_menu.hpp"

#include <string.h>

#include "ws19804.hpp"
#include "mono8x16.hpp"
#include "common.hpp"

#include "ff.h"
#include "diskio.h"

constexpr int MAX_FILES = 64;

// enumerate NES files
static int enum_files(FATFS *fs, char **fname_list, int *fsize_list);

// show ROM select menu
static int rom_select(int num_files, char **file_list);

// load NES file
static bool load_nes(const char *fname, int size);

bool boot_menu() {
    FATFS fs;
    char *fname_list[MAX_FILES];
    int fsize_list[MAX_FILES];
    int num_files = enum_files(&fs, fname_list, fsize_list);
    if (num_files < 0) {
        return false;
    }

    int index = rom_select(num_files, fname_list);

    if ( ! load_nes(fname_list[index], fsize_list[index])) {
        return false;
    }

    for (int i = 0; i < num_files; i++) {
        free(fname_list[i]);
    }
    
    return true;
}

static int enum_files(FATFS *fs, char **fname_list, int *fsize_list) {
    char tmp[16];
    int y = 20;
    constexpr int x_result = 120;

    DSTATUS dsts;
    FRESULT fres;

    clear_frame_buff();

    // Init FatFS
    draw_string(0, y, "Init FatFS"); update_lcd();
    dsts = disk_initialize(0);
    if (dsts & STA_NOINIT) {
        sprintf(tmp, "[NG] code=0x%x", (int)dsts);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    // Mount
    draw_string(0, y, "Disk Mount"); update_lcd();
    fres = f_mount(fs, "", 0);
    if (fres != FR_OK) {
        sprintf(tmp, "[NG] code=0x%x", (int)fres);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    // Enumerate File
    draw_string(0, y, "File List"); update_lcd();

    DIR dobj;
    FILINFO finfo;
    fres = f_findfirst(&dobj, &finfo, "", "*.nes");
    int num_files = 0;
    while (fres == FR_OK && finfo.fname[0]) {
        fname_list[num_files] = (char*)malloc(strlen(finfo.fname) + 1);
        strcpy(fname_list[num_files], finfo.fname);
        fsize_list[num_files] = finfo.fsize;
        fres = f_findnext(&dobj, &finfo);
        num_files++;
    }
    if (fres != FR_OK) {
        sprintf(tmp, "[NG] code=0x%x", (int)fres);
        draw_string(x_result, y, tmp); update_lcd();
        return -1;
    }
    f_closedir(&dobj);
    draw_string(x_result, y, "[OK]"); update_lcd();
    y += 20;

    return num_files;
}

static int rom_select(int num_files, char **file_list) {
    int sel_index = 0;

    for(;;) {
        clear_frame_buff();
        for(int i = 0; i < num_files; i++) {
            draw_string(20, i * 20, file_list[i]);
        }
        draw_string(0, sel_index * 20, "=>");
        update_lcd();

        switch(wait_key()) {
        case nes::input::BTN_UP:
            sel_index = (sel_index + num_files - 1) % num_files;
            break;
        case nes::input::BTN_DOWN:
            sel_index = (sel_index + 1) % num_files;
            break;
        case nes::input::BTN_A:
        case nes::input::BTN_START:
            return sel_index;
        }
    }
}

static bool load_nes(const char *fname, int size) {
    FIL fil;
    FRESULT fr;

    clear_frame_buff();
    draw_string(0, 0, "Loading...");
    update_lcd();

    fr = f_open(&fil, fname, FA_READ);
    if (fr) {
        draw_string(0, 20, "File open failed.");
        update_lcd();
        return false;
    }

    UINT sz;
    uint8_t *ines = (uint8_t*)malloc(size);
    fr = f_read(&fil, ines, size, &sz);
    if (fr) {
        draw_string(0, 20, "File read failed.");
        update_lcd();
        return false;
    }

    f_close(&fil);

    nes::memory::map_ines(ines);
    ws19804::set_spi_speed(SYS_CLK_FREQ / 4);

    return true;
}
