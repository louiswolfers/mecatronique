#include <avr/io.h>
#include <util/delay.h>

const unsigned long Measure_Timeout = 25000UL;
float temperature;
float temps; // Pas besoin d'initier une variable
float vitesse;
float vitesseDuSon = 0.0034; 
// On ne l'utilise qu'une fois, autant le diviser par 1000 directement
float distance1 = 0.0;
float distance2 = 0.0;
float temps1 = 0.0;
float temps2 = 0.0;

// Code de M. Perreti
uint16_t ReadADC(uint8_t __channel){ 
	/*
	 * Selection de l'entrée analogique à convertir.
	 * On met les 4 bits de poids faible à 0 pour les recharger
	 * via le OU à la valeur __channel. 
	 */
	ADMUX = (ADMUX & 0xf0) | __channel; 
	/*
	 * Début de conversion
	 */
	ADCSRA |= (1 << ADSC);
	/*
	 * Attente de la fin de conversion:
	 * A la fin de la conversion, ADIF passe à 1, c'est le bit 4
	 * de ADCSRA, donc on fait un & avec 0b00010000 = 0x10 
	 */
	while((ADCSRA & 0x10) != 0x10);
	/*
	 * Dans la datasheet il est dit que le reset se fait en mettant un 1 logique
	 * dans le flag.
	 * "Alternatively, ADIF is cleared by writing a logical one to the flag." 
	 */
	ADCSRA |= (1 << ADIF);
	return(ADC); 
}

float conversion(uint8_t pin) {
	uint16_t valeur = ReadADC(pin);
	return (valeur * 0.107422) - 50;
}

int main(void) {
	ADMUX |= (1<< REFS1);
	// On défini le voltage de réference comme 1,1V
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA &= ~(1 << ADATE);
	DDRC &= ~(1 << PIN0);
	// On défini PC0 comme une entrée
	DDRD |= (1 << PIN0);
	// On défini PD0 comme une sortie
	DDRD |= (1 << PIN2);
	// On défini PD2 comme une sortie
	DDRD |= (1 << PIN3);
	// On défini PD3 comme une sortie
	PORTD |= (1 << PIN0);
	// On envoie un signal HIGH à PD0
	DDRB |= (1 << PIN2);
	// On défini PB2 comme une sortie
	DDRB &= ~(1 << PIN3);
	// On défini PB3 comme une entrée
	DDRD |= (1 << PIN5);
	// On défini PD5 comme une sortie
	DDRD &= ~(1 << PIN6);
	// On défini PD6 comme une entrée
	Serial.begin(9600);
	for(;;) {
		temperature = conversion(0);
		// On lit le signal analogue venant de A0 et on le converti en signal digital
		vitesse = (float)((-0.0006*temperature*temperature + 0.6069*temperature + 331.52)/100.0);

		PORTB |= (1 << PIN2);
		// On envoie un signal HIGH à PB2
		_delay_ms(1);
		// On attends 10 microsecondes
		PORTB &= ~(1 << PIN2);
		// On envoie un signal LOW à PB2
		temps1 = pulseIn(11, HIGH, Measure_Timeout) * 1000;
		distance1 = (float) (temps1 / 2.0 * (vitesse / 1000.0));

		PORTD |= (1 << PIN5);
		// On envoie un signal HIGH à PB2
		_delay_ms(1);
		// On attends 10 microsecondes
		PORTD &= ~(1 << PIN5);
		// On envoie un signal LOW à PB2
		temps2 = pulseIn(6, HIGH, Measure_Timeout) * 1000;
		distance2 = (float) (temps2 / 2.0 * (vitesse / 1000.0));

		Serial.println("Temperature mesuree: ");
		Serial.print(temperature)
		Serial.println("degres C");

		Serial.print(temps1);
		Serial.println("microseconds");
		Serial.println(distance1 / 100.0);
		Serial.println("cm");
		Serial.print(temps2);
		Serial.println("microseconds");
		Serial.println(distance2 / 100.0);
		Serial.println("cm");

		_delay_ms(1000);
		if (distance1 <= 4/100.0) {
			if (distance2 - diff_longueur < 3/100.0) {
				PORTD |= (1 << PIN2);
				// On envoie un signal HIGH à PB2
				PORTD &= ~(1 << PIN3);
				// On envoie un signal LOW à PB2
				_delay_ms(1000);
			} else {
			}
		} else if (distance1 <= 1/10.0) {
		} else {
			PORTD &= ~(1 << PIN2);
			// On envoie un signal LOW à PD2
			PORTD |= (1 << PIN3);
			// On envoie un signal HIGH à PD3
		}
	}
	return 0;
}

