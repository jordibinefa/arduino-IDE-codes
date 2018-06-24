#define LORA_MODE_ABP   0
#define LORA_MODE_OTAA  1

#define LORA_MODE LORA_MODE_ABP

#if LORA_MODE == LORA_MODE_ABP
  /* NWKSKEY, APPSKEY and DEVADDR should by modifed by your actual values */
  //static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u4_t DEVADDR = 0x00000000 ;

  static const PROGMEM u1_t NWKSKEY[16] = {  };
  static const u1_t PROGMEM APPSKEY[16] = {  };
  static const u4_t DEVADDR = 0x00000000 ;
  
#endif

#if LORA_MODE == LORA_MODE_OTAA
    // De moment no és operatiu a aquesta versió
    const char *appEui = "";
    const char *appKey = "";
#endif
