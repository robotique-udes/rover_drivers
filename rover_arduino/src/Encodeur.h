#ifndef ENCODEUR_H
#define ENCODEUR_H

#include <Arduino.h>
#include "gestionaireTimer2.h"
class Encodeur
{
protected:
    static float _position[10];
    static int _derniereLecture[10];
    static int _pin[10];

    static bool initialisation;
    static int nbr;
    static void update();

    int id;

public:
    Encodeur(int pin);
    float ReadAngle();
};


#endif