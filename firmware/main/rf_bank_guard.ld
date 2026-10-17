/* The dump engine owns the whole 128 KiB RF bank while a capture runs. */
ASSERT(_heap_start <= 0x3fca0000, "Static data overlaps the RF dump bank; reduce BSS usage")
