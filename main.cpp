
//#include "src/tools.h"
#include "experience.h"
#include <iostream>


int main()
{

//std::cout << Solver::solve(1., -2., 1, -1.) <<std::endl;

    Experience exp(0., 10., 5., 1., 100., 100., -100., -100.);
    std::cout << exp.getYawP()*180./3.141 << std::endl;
    std::cout << exp.getXP() << std::endl;
    std::cout << exp.getYawP()*180./3.141 << std::endl;
    std::cout << exp.getXP() << std::endl;
    std::cout << exp.getYawP()*180./3.141 << std::endl;
    std::cout << exp.getXP() << std::endl;


    return 0;

}


//CEST LE 1ER ROBOT QUI MENE LES AUTRES
