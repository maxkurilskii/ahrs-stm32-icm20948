#include <errno.h>
#include <sys/types.h>

extern char end;       /* определён в .ld через PROVIDE */
extern char _estack;   /* верх RAM, определён в .ld */

caddr_t _sbrk(int incr)
{
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
        heap_end = &end;

    prev_heap_end = heap_end;

    if (heap_end + incr > &_estack) {
        errno = ENOMEM;
        return (caddr_t) -1;
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}