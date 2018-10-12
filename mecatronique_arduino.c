const unsigned long Measure_Timeout = 25000UL;
int capteur_temp = A0; // Pin analogique pour le capteur de temperature
int trigger1 = 10; // Pin du trigger du premier capteur
int echo1 = 11; // Pin du echo du premier capteur
int trigger2 = 5; // Pin du trigger du deuxieme capteur
int echo2 = 6; // Pin du echo du deuxieme capteur
int l293d_enable = 0;
int l293d_1a = 2;
int l293d_2a = 3;
int diff_longueur = 3; // Difference de longueur entre les deux capteurs
int distance1 = 0;
int distance2 = 0;
float temperature = 0.0;
float vitesse = 0.0;
float temps1 = 0.0;
float temps2 = 0.0;

void setup()
{
	pinMode(l293d_enable, OUTPUT);
	pinMode(l293d_1a, OUTPUT);
	pinMode(l293d_2a, OUTPUT);
	digitalWrite(l293d_enable, HIGH);
	analogReference(INTERNAL);

	pinMode(trigger1, OUTPUT);
	pinMode(trigger2, OUTPUT);
	pinMode(echo1, INPUT);
	pinMode(echo2, INPUT);
	pinMode(capteur_temp, INPUT);
	digitalWrite(echo, LOW);
	Serial.begin(9600);
}

void loop()
{
	temperature = conversion0(capteur_temp);
	vitesse = (float)(0.0017*temperature0*temperature0 - 0.1208*temperature0 + 185.97);

	temps1 = obtenir_temps(echo1, trigger1);
	distance1 = (float)(temps1 / 2.0 * (vitesse / 1000.0));

	temps2 = obtenir_temps(echo2, trigger2);
	distance2 = (float)(temps2 / 2.0 * (vitesse / 1000.0));

	Serial.println("Temperature mesuree: ");
	Serial.print(temperature);
	Serial.println("degres C");

	Serial.print(distance1/100.0);
	Serial.println(" cm");
	Serial.print(distance2/100.0);
	Serial.println(" cm");

	delay(2000);
	if (distance1 <= 4/100.0)
	{
		if (distance2 - difflongueur < 3/100.0)
		{
			digitalWrite(l293d_1a, HIGH);
			digitalWrite(l293d_2a, LOW);
			delay(1000);
		}
		else
		{
			analogWrite(l293d_2a, (int)(255.0/7.0 * (distance2 - difflongueur) - 765.0/7.0));
		}
	} 
	else if (distance1 <= 1/10.0) 
	{
		analogWrite(l293d_2a, (int)(255.0/7.0 * distance - 765.0/7.0));
	} 
	else 
	{
		digitalWrite(l293d_1a, LOW);
		digitalWrite(l293d_2a, HIGH);
	}
}

float conversion(float valeur_capteur_temp)
{
	valeur_capteur_temp = analogRead(capteurtemp);
	float temperature = (float)(valeur_capteur_temp * 1100.0/1024-500) / 10; //temperature en tension
	return temperature;
}

float obtenir_temps(int echo, int trigger)
{
	digitalWrite(trigger, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigger, LOW);
	return pulseIn(echo, HIGH, Measure_Timeout) * 1000.0;
}
