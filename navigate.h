const float Pi = 3.14159265;  

static float ConvertToRadians(float angle)
{
  return (Pi / 180) * angle;
}

static float haversin(float input)
{
  float sinn;
  sinn = sin(input / 2);
  return (sinn * sinn);
}

static float GetDistanceInM(float lat1, float long1, float lat2, float long2)
{
  lat1 = ConvertToRadians(lat1);
  lat2 = ConvertToRadians(lat2);
  long1 = ConvertToRadians(long1);
  long2 = ConvertToRadians(long2);
  float dLat = lat2 - lat1;
  float dLon = long2 - long1;
  float a = haversin(dLat) + cos(lat1) * cos(lat2) * haversin(dLon);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000 * c ; // Distance in M
}

static float ConvertToStaticDegrees(float rad)
{
  float degrees = rad / (Pi / 180);
  if (degrees <= 0) {
    return degrees + 360;
  }
  else {
    return degrees;
  }
}

static float GetHeading(float lat1, float long1, float lat2, float long2)
{
  //To get reverse heading, add 180
  lat1 = ConvertToRadians(lat1);
  lat2 = ConvertToRadians(lat2);
  long1 = ConvertToRadians(long1);
  long2 = ConvertToRadians(long2);
  float dLon = long2 - long1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  return ConvertToStaticDegrees(atan2(y, x));
}

/*

  float a1=49.435984;
  float b1=27.102571;

  float a2=49.4318;
  float b2=27.110167;
  
  Serial.print("Distance=");
  Serial.println( GetDistanceInM(a1,b1,a2,b2) );

  Serial.print("Heading=");
  Serial.println( GetHeading(a2,b2,a1,b1) );


*/

