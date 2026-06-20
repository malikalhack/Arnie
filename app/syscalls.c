/**
 * @file    syscalls.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-03
 * @date    @showdate "%Y-%m-%d"
 */

/*
 * Minimal newlib syscall stubs for bare-metal GCC builds.
 *
 * newlib (libc_nano.a) references _close, _lseek, _read and _write through
 * its internal I/O cleanup path even when printf / stdio are not used.
 * Without explicit definitions the linker falls back to the built-in stubs
 * that emit "will always fail" warnings at link time.
 *
 * Providing strong definitions here silences those warnings without pulling
 * in any additional library code.  The stubs are intentionally no-ops:
 * this project drives UART output directly through CMSIS register writes and
 * never calls any stdio function.
 *
 * These stubs must NOT be compiled for AC6 (armclang) because the ARM C
 * Library uses a different retargeting mechanism (fputc / __write).
 * Include this file only in a GCC-only build group.
 */

/****************************** Private functions *****************************/

/** @fn _close */
int _close(int fd) {
    (void)fd;
    return -1;
}
/*----------------------------------------------------------------------------*/

/** @fn _lseek */
int _lseek(int fd, int ptr, int dir) {
    (void)fd;
    (void)ptr;
    (void)dir;
    return 0;
}
/*----------------------------------------------------------------------------*/

/** @fn _read */
int _read(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    (void)len;
    return 0;
}
/*----------------------------------------------------------------------------*/

/** @fn _write */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    (void)ptr;
    return len;
}
/******************************************************************************/
