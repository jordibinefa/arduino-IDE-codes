#define LORA_MODE_ABP   0
#define LORA_MODE_OTAA  1

#define LORA_MODE LORA_MODE_ABP

#if LORA_MODE == LORA_MODE_ABP
  /* NWKSKEY, APPSKEY and DEVADDR should by modifed by your actual values */
  //static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u4_t DEVADDR = 0x00000000 ;

  static const PROGMEM u1_t NWKSKEY[16] = { 0xD1, 0xB8, 0xB5, 0xFF, 0x38, 0xCD, 0x23, 0x98, 0xB2, 0xBD, 0xDD, 0x94, 0xC4, 0xF7, 0x28, 0xF0 };
  static const u1_t PROGMEM APPSKEY[16] = { 0x38, 0xB7, 0xE9, 0x2C, 0x6C, 0x51, 0x3A, 0xD4, 0x3A, 0x0F, 0xB5, 0x5B, 0xEA, 0x31, 0xBE, 0x44 };
  static const u4_t DEVADDR = 0x26011CE7;
  
#endif

#if LORA_MODE == LORA_MODE_OTAA
    // De moment no és operatiu a aquesta versió
    const char *appEui = "";
    const char *appKey = "";
#endif
