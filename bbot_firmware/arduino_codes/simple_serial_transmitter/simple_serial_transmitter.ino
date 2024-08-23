int num_joints = 8;
void setup(){
  Serial.begin(115200);
  while(!Serial);
}

void loop(){
  if (Serial.available())
  {
    char c = Serial.read();
    if(c == 'e\r')
    {
      float j[] = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
      Serial.write((char*)j, num_joints * sizeof(float));
    }
  }
}