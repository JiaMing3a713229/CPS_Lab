
const uint64_t p = 1549;
const uint64_t g = 19;


uint32_t gcd(uint32_t a, uint32_t b){   //gcd(a,b)=> gcd(b,a mod b)
  if(b == 0){
    return a;
  }
  uint32_t c = a % b;
  return gcd(b, c);
}

uint64_t MODEXP(uint64_t a, uint64_t n, uint64_t m){
  a = a % m;  
  uint64_t r = 1;
  while(n!=0){
    if((n & 1) == 1){
      r = (a % m)*(r % m) % m;
    }
    a = (a % m)*(a % m) % m ;
    n = n >> 1;
  }
  return r;
}

uint64_t exgcd(uint64_t a, uint64_t b, uint64_t a1, uint64_t t1, uint64_t t2){
  if(b==0){
    return (t1 + a1) % a1;
  }
  uint64_t Q = int(a / b);
  uint64_t R = a % b;
  uint64_t t = t1 - (t2 * Q);
  t1 = t2;
  t2 = t;
  a = b;
  b = R;
  return exgcd(a, b, a1, t1, t2);
}
uint64_t InverseModular(uint64_t k,uint64_t n){
  uint64_t r = exgcd(n, k, n, 0, 1);
  return r;
}




void setup() {
  

  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  static uint64_t m = 0;

  uint64_t y = 21;
  // bob : sned (p,g,Y), Y = g^y mod p
  uint64_t Y = MODEXP(g, y, p);

  // alice receive Y, K = Y^x mod p, c = K*m mod p
  uint64_t x = 17;
  
  uint64_t K = MODEXP(Y, x, p);

  while(m<256){
    m++;
    //Encrypyion
    uint64_t c = (K*m) % p;

    //Decryption
    uint64_t plaintext = (c * InverseModular(K, p)) % p;

    Serial.printf("message:%5d,", m);
    Serial.printf("cipher:%5d,", c);
    Serial.printf("plaintext:%5d \r\n", plaintext);
    Serial.println("------------------------------------------");
  }

  //Encrypyion
  // uint64_t c = (K*m) % p;

  // //Decryption
  // uint64_t plaintext = (c * InverseModular(K, p)) % p;

  // Serial.printf("message:%d,", m);
  // Serial.printf("cipher:%d,", c);
  // Serial.printf("plaintext:%d \r\n", plaintext);


}

void loop() {
  // put your main code here, to run repeatedly:

}
