#include "image.h"

struct img_head ih =
{
magic: 	{ 'A','I','H' },
hlen: 	0x30,
time: 	IH_TIME ,
run: 	IH_RUN ,
size: 	IH_SIZE ,
mid:	IH_MID,
ver:	IH_VER,
desc: 	"ip3210 linux"
} ;
