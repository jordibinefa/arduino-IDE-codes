#define N_WIFIS 3
#define MAX_STRING_SIZE 15

struct stWifiList {
  const char* szSSID;
  const char* szPWD;
};

struct stWifiList stWiFi[N_WIFIS] = {
  {"iotcat" , "1234567890"},
  {"IoT-eCat" , "clotClot"},
  {"JESUITESFP" , "internetcoses"}
};

