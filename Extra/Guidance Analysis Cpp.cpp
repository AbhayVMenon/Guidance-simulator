#include <iostream>
#include <vector>


int main(){

std::vector <int>x {69, 420, 1976};
std::cout << x[1] << "\n";
std::cout << &x[1] << "\n"; //0x294e8649074
std::cout << *&x[1] << "\n";

}
