
#define ETHERNET_PHY_ADDR 0

// National DP83846A PHY definitions

// Registers
#define PHY_BMCR      0x00
    #define GET_BMCR_DUPLEX(x) ((x>>8)&0x1)
    #define SET_BMCR_DUPLEX(upd) (upd<<8)
    #define GET_BMCR_RESTART_AUTONEG(x) ((x>>9)&0x1)
    #define SET_BMCR_RESTART_AUTONEG(upd) (upd<<9)
    #define GET_BMCR_ISOLATE(x) ((x>>10)&0x1)
    #define SET_BMCR_ISOLATE(upd) (upd<<10)
    #define GET_BMCR_AUTONEG(x) ((x>>12)&0x1)
    #define SET_BMCR_AUTONEG(upd) (upd<<12)
    #define GET_BMCR_SPEED(x) ((x>>13)&0x1)
    #define SET_BMCR_SPEED(upd) (upd<<13)
    #define GET_BMCR_LOOPBACK(x) ((x>>14)&0x1)
    #define SET_BMCR_LOOPBACK(upd) (upd<<14)
    #define GET_BMCR_RESET(x) ((x>>15)&0x1)
    #define SET_BMCR_RESET(upd) (upd<<15)
#define PHY_PHYIDR1   0x02
#define PHY_PHYIDR2   0x03
#define PHY_PHYSTS    0x10
    #define GET_PHYSTS_LOOPBACK(x) ((x>>3)&0x1)
    #define SET_PHYSTS_LOOPBACK(upd) (upd<<3)
#define PHY_PHYCTRL   0x19

