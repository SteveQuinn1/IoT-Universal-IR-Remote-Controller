#ifndef REMOTE_CONTROL_H
#define  REMOTE_CONTROL_H

#ifdef  __cplusplus
extern "C" {
#endif

typedef enum {
   eREMOTE_CONTROL_TYPES_SKY    = 0,
   eREMOTE_CONTROL_TYPES_SONYSB = 1,
   eREMOTE_CONTROL_TYPES_SONYTV = 2,
   eREMOTE_CONTROL_TYPES_SKYQ   = 3
} eREMOTE_CONTROL_TYPES;

typedef enum {
   eREMOTE_CONTROL_CODING_SCHEME_RC6     = 0,
   eREMOTE_CONTROL_CODING_SCHEME_SONY    = 1,
   eREMOTE_CONTROL_CODING_SCHEME_SAMSUNG = 2,
   eREMOTE_CONTROL_CODING_SCHEME_NEC     = 3,
   eREMOTE_CONTROL_CODING_SCHEME_LG      = 4,
   eREMOTE_CONTROL_CODING_SCHEME_MIN     = eREMOTE_CONTROL_CODING_SCHEME_RC6,
   eREMOTE_CONTROL_CODING_SCHEME_MAX     = eREMOTE_CONTROL_CODING_SCHEME_SAMSUNG
} eREMOTE_CONTROL_CODING_SCHEME;


typedef struct __attribute((__packed__)) {
  char *sButtonName;
  byte bEncoding;
  uint32_t ui32Data;
  byte bNumberOfBitsInTheData;
  byte bPulseTrainRepeats;
  byte bDelayBetweenPulseTrainRepeats;
  byte bButtonRepeats;
  uint16_t ui16DelayBetweenButtonRepeats;
  byte bFreshData;
} remoteControlButtonType;


typedef struct __attribute((__packed__)) {
  char *sRemoteName;
  byte bRemoteName;
  uint16_t ui16ModulationFrequency;
  remoteControlButtonType *remoteControlButtonCollection;
  uint16_t sizeOfRemoteControlButtonCollection;
} remoteControlType;


//remoteControlButtonType remoteControlButtonCollectionSky[] PROGMEM = {
remoteControlButtonType remoteControlButtonCollectionSky[] = {
  {(char *) "Sky",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C80, (byte)24, (byte)3, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "VolUp",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x490,    (byte)12, (byte)3, (byte)26,  (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "VolDn",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xC90,    (byte)12, (byte)3, (byte)26,  (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Up",      (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C58, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Dn",      (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C59, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Left",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C5A, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Right",   (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C5B, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Select",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C5C, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Backup",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C83, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Pause",   (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C24, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Play",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C3E, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Record",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C40, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Stop",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C3F, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Rewind",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C3D, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "FastFwd", (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C28, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "1",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C01, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "2",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C02, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "3",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C03, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "4",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C04, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "5",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C05, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "6",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C06, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "7",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C07, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "8",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C08, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "9",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C09, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "0",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C00, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "ChanUp",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C20, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "ChanDn",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C21, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Red",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C6D, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Green",   (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC056CE, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Yellow",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C6F, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Blue",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C70, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "OnOff",   (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C0C, (byte)24, (byte)3, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Info",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05CCB, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "TVGuide", (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC05C00, (byte)24, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00}
};

//remoteControlButtonType remoteControlButtonCollectionSonySB[] PROGMEM = {
remoteControlButtonType remoteControlButtonCollectionSonySB[] = {
  {(char *) "OnOff",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x540C, (byte)15, (byte)4, (byte)22, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "TVOnOff", (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xA90,  (byte)12, (byte)2, (byte)25, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Std",     (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xB8116,(byte)20, (byte)4, (byte)14, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Movie",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x610D, (byte)15, (byte)4, (byte)22, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Mute",    (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x140C, (byte)15, (byte)4, (byte)22, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "SndUp",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x240C, (byte)15, (byte)4, (byte)22, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "SndDn",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x640C, (byte)15, (byte)4, (byte)22, (byte)1, (uint16_t)400, (byte)0x00}
};

//remoteControlButtonType remoteControlButtonCollectionSonyTV[] PROGMEM = {
remoteControlButtonType remoteControlButtonCollectionSonyTV[] = {
  {(char *) "OnOff", (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xA90, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "SelIP", (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xA50, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Up",    (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x2F0, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Dn",    (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xAF0, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Left",  (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0x2D0, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Right", (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xCD0, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Sel",   (byte)eREMOTE_CONTROL_CODING_SCHEME_SONY, (uint32_t)0xA70, (byte)12, (byte)4, (byte)26, (byte)1, (uint16_t)400, (byte)0x00}
};

//remoteControlButtonType remoteControlButtonCollectionSkyQ[] PROGMEM = {
remoteControlButtonType remoteControlButtonCollectionSkyQ[] = {
  {(char *) "Sky",        (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A80, (byte)32, (byte)3, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "OnOff",      (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A0C, (byte)32, (byte)3, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Search",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A7E, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Rewind",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A3D, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "PlayPause",  (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A3E, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Forward",    (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A28, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Up",         (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A58, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Dn",         (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A59, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Left",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A5A, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Right",      (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A5B, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Select",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A5C, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "BackReturn", (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A83, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Home",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081ACC, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Ellipsis",   (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081AF5, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Info",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081ACB, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "ChanUp",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A20, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "ChanDn",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A21, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Record",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A40, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Red",        (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A6D, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Green",      (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A6E, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Yellow",     (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A6F, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Blue",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A70, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "1",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A01, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "2",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A02, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "3",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A03, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "4",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A04, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "5",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A05, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "6",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A06, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "7",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A07, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "8",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A08, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "9",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A09, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "0",          (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A00, (byte)32, (byte)2, (byte)124, (byte)1, (uint16_t)400, (byte)0x00},
  {(char *) "Help",       (byte)eREMOTE_CONTROL_CODING_SCHEME_RC6,  (uint32_t)0xC0081A81, (byte)32, (byte)3, (byte)124, (byte)1, (uint16_t)400, (byte)0x00}
};

//remoteControlType remoteControls[] PROGMEM = {
remoteControlType remoteControls[] = {
  {
    (char *)"Sky",
    (byte) eREMOTE_CONTROL_TYPES_SKY,
    (uint16_t)36, // not used
    (remoteControlButtonType *) remoteControlButtonCollectionSky,
    (uint16_t) (sizeof(remoteControlButtonCollectionSky)/sizeof(remoteControlButtonType))
  },
  {
    (char *)"SonySB",
    (byte) eREMOTE_CONTROL_TYPES_SONYSB,
    (uint16_t)40, // not used
    (remoteControlButtonType *) remoteControlButtonCollectionSonySB,
    (uint16_t) (sizeof(remoteControlButtonCollectionSonySB)/sizeof(remoteControlButtonType))
  },
  {
    (char *)"SonyTV",
    (byte) eREMOTE_CONTROL_TYPES_SONYTV,
    (uint16_t)40, // not used
    (remoteControlButtonType *) remoteControlButtonCollectionSonyTV,
    (uint16_t) (sizeof(remoteControlButtonCollectionSonyTV)/sizeof(remoteControlButtonType))
  },
  {
    (char *)"SkyQ",
    (byte) eREMOTE_CONTROL_TYPES_SKYQ,
    (uint16_t)36, // not used
    (remoteControlButtonType *) remoteControlButtonCollectionSkyQ,
    (uint16_t) (sizeof(remoteControlButtonCollectionSkyQ)/sizeof(remoteControlButtonType))
  }
};


uint16_t maxRemoteControls = (sizeof(remoteControls)/sizeof(remoteControlType));


#ifdef  __cplusplus
}
#endif

#endif  /* REMOTE_CONTROL_H */
