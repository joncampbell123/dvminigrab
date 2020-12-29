/* libiec61883 private stuff.
 * would be nice though if this were public rather than having to dig it out, fuckers >:( */

/************ DECLARE PRIVATE INTERFACE BITS OF LIBIEC61883!! **************/

/* standard CSR offsets for plugs */
#define CSR_O_MPR   0x900
#define CSR_O_PCR_0 0x904
#define CSR_O_PCR_1 0x908
#define CSR_O_PCR_2 0x90C
#define CSR_O_PCR_3 0x910

#define CSR_I_MPR   0x980
#define CSR_I_PCR_0 0x984
#define CSR_I_PCR_1 0x988
#define CSR_I_PCR_2 0x98C
#define CSR_I_PCR_3 0x990

#if ( __BYTE_ORDER == __BIG_ENDIAN )

struct iec61883_oMPR {
	unsigned int data_rate:2;
	unsigned int bcast_channel:6;
	unsigned int non_persist_ext:8;
	unsigned int persist_ext:8;
	unsigned int reserved:3;
	unsigned int n_plugs:5;
};

struct iec61883_iMPR {
	unsigned int data_rate:2;
	unsigned int reserved:6;
	unsigned int non_persist_ext:8;
	unsigned int persist_ext:8;
	unsigned int reserved2:3;
	unsigned int n_plugs:5;
};

struct iec61883_oPCR {
	unsigned int online:1;
	unsigned int bcast_connection:1;
	unsigned int n_p2p_connections:6;
	unsigned int reserved:2;
	unsigned int channel:6;
	unsigned int data_rate:2;
	unsigned int overhead_id:4;
	unsigned int payload:10;
};

struct iec61883_iPCR {
	unsigned int online:1;
	unsigned int bcast_connection:1;
	unsigned int n_p2p_connections:6;
	unsigned int reserved:2;
	unsigned int channel:6;
	unsigned int reserved2:16;
};

#else

struct iec61883_oMPR {
	unsigned int n_plugs:5;
	unsigned int reserved:3;
	unsigned int persist_ext:8;
	unsigned int non_persist_ext:8;
	unsigned int bcast_channel:6;
	unsigned int data_rate:2;
};

struct iec61883_iMPR {
	unsigned int n_plugs:5;
	unsigned int reserved2:3;
	unsigned int persist_ext:8;
	unsigned int non_persist_ext:8;
	unsigned int reserved:6;
	unsigned int data_rate:2;
};

struct iec61883_oPCR {
	unsigned int payload:10;
	unsigned int overhead_id:4;
	unsigned int data_rate:2;
	unsigned int channel:6;
	unsigned int reserved:2;
	unsigned int n_p2p_connections:6;
	unsigned int bcast_connection:1;
	unsigned int online:1;
};

struct iec61883_iPCR {
	unsigned int reserved2:16;
	unsigned int channel:6;
	unsigned int reserved:2;
	unsigned int n_p2p_connections:6;
	unsigned int bcast_connection:1;
	unsigned int online:1;
};

#endif

/**
 * iec61883_plug_get - Read a node's plug register.
 * @h: A raw1394 handle.
 * @n: The node id of the node to read
 * @a: The CSR offset address (relative to base) of the register to read.
 * @value: A pointer to a quadlet where the plug register's value will be stored.
 * 
 * This function handles bus to host endian conversion. It returns 0 for 
 * suceess or -1 for error (errno available).
 **/
extern int
iec61883_plug_get(raw1394handle_t h, nodeid_t n, nodeaddr_t a, quadlet_t *value);


/** 
 * iec61883_plug_set - Write a node's plug register.
 * @h: A raw1394 handle.
 * @n: The node id of the node to read
 * @a: The CSR offset address (relative to CSR base) of the register to write.
 * @value: A quadlet containing the new register value.
 *
 * This uses a compare/swap lock operation to safely write the
 * new register value, as required by IEC 61883-1.
 * This function handles host to bus endian conversion. It returns 0 for success
 * or -1 for error (errno available).
 **/
extern int
iec61883_plug_set(raw1394handle_t h, nodeid_t n, nodeaddr_t a, quadlet_t value);

/**
 * High level plug access macros
 */

#define iec61883_get_oMPR(h,n,v) iec61883_plug_get((h), (n), CSR_O_MPR, (quadlet_t *)(v))
#define iec61883_set_oMPR(h,n,v) iec61883_plug_set((h), (n), CSR_O_MPR, *((quadlet_t *)&(v)))
#define iec61883_get_oPCR0(h,n,v) iec61883_plug_get((h), (n), CSR_O_PCR_0, (quadlet_t *)(v))
#define iec61883_set_oPCR0(h,n,v) iec61883_plug_set((h), (n), CSR_O_PCR_0, *((quadlet_t *)&(v)))
#define iec61883_get_oPCRX(h,n,v,x) iec61883_plug_get((h), (n), CSR_O_PCR_0+(4*(x)), (quadlet_t *)(v))
#define iec61883_set_oPCRX(h,n,v,x) iec61883_plug_set((h), (n), CSR_O_PCR_0+(4*(x)), *((quadlet_t *)&(v)))
#define iec61883_get_iMPR(h,n,v) iec61883_plug_get((h), (n), CSR_I_MPR, (quadlet_t *)(v))
#define iec61883_set_iMPR(h,n,v) iec61883_plug_set((h), (n), CSR_I_MPR, *((quadlet_t *)&(v)))
#define iec61883_get_iPCR0(h,n,v) iec61883_plug_get((h), (n), CSR_I_PCR_0, (quadlet_t *)(v))
#define iec61883_set_iPCR0(h,n,v) iec61883_plug_set((h), (n), CSR_I_PCR_0, *((quadlet_t *)&(v)))
#define iec61883_get_iPCRX(h,n,v,x) iec61883_plug_get((h), (n), CSR_I_PCR_0+(4*(x)), (quadlet_t *)(v))
#define iec61883_set_iPCRX(h,n,v,x) iec61883_plug_set((h), (n), CSR_I_PCR_0+(4*(x)), *((quadlet_t *)&(v)))

/************ END PRIVATE INTERFACE **************/

