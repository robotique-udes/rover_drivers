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

int gestionaireTimer2::taille = 0;
void(*gestionaireTimer2::fonctions[5])() {gestionaireTimer2::f0,gestionaireTimer2::f1,gestionaireTimer2::f2,gestionaireTimer2::f3,gestionaireTimer2::f4};
void gestionaireTimer2::f0(){;}
void gestionaireTimer2::f1(){;}
void gestionaireTimer2::f2(){;}
void gestionaireTimer2::f3(){;}
void gestionaireTimer2::f4(){;}

void gestionaireTimer2::lancer()
{
    int i = 0;
    while (i < taille)
    {
        (*fonctions[i++])();
    }
    
}

void gestionaireTimer2::ajouter(void(*nouvelleFonction)())
{
    MsTimer2::stop();
    if (taille < 5)
    {
        fonctions[taille]=nouvelleFonction;
        taille ++;
    }
    MsTimer2::set(5,lancer);
    MsTimer2::start();
}


#endif