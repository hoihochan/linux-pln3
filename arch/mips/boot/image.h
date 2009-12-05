
struct img_head
{
	unsigned char magic[3];
	unsigned char hlen;	//header length
	unsigned int time;	// time when building image
	unsigned int run;	// where to run
	unsigned int size;	// image size

	unsigned int next;	// chain to next?
	unsigned short flags;	// flags
	unsigned short mid;	// model
	unsigned int ver;	// ver info 
	unsigned int resv;	// 

	char desc[16];
}__attribute__((packed)) ; 

#define	IH_MAGIC 	{ 'A', 'I', 'H' }
