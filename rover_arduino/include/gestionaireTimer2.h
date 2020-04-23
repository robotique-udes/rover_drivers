#ifndef GESTIONAIRETIMER2_H
#define GESTIONAIRETIMER2_H

#include <MsTimer2.h>

class gestionaireTimer2
{
private:
    static void f0();
    static void f1();
    static void f2();
    static void f3();
    static void f4();
    static void(*fonctions[5])();
    static int taille ;
public:
    gestionaireTimer2();
    static void lancer();
    static void ajouter(void(*g)());
};

#endif