#include <iostream>

class mclass
{
    mclass()
    {
        std::cout << "constructor\n";
    }

    // mclass(const mclass &mc) = delete;

};

int main()
{
    mclass mc1();


}