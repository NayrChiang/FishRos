#include <iostream>
#include <memory>

int main()
{
    auto p1 = std::make_shared<std::string>("This is a string.");
    std::cout << "p1 reference counter: " << p1.use_count() << ", point memory address: " << p1.get() << std::endl; // 1

    auto p2 = p1;
    std::cout << "p1 reference counter: " << p1.use_count() << ", point memory address: " << p1.get() << std::endl; // 2
    std::cout << "p2 reference counter: " << p2.use_count() << ", point memory address: " << p2.get() << std::endl; // 2

    p1.reset();
    std::cout << "p1 reference counter: " << p1.use_count() << ", point memory address: " << p1.get() << std::endl; // 0
    std::cout << "p2 reference counter: " << p2.use_count() << ", point memory address: " << p2.get() << std::endl; // 2-1 = 1

    std::cout << "p2 pointer's' data: " << p2->c_str() << std::endl;
    return 0;
}
