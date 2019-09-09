#include <iostream>


// easy debugging preprocessors
#define OUT_INFO(__X__) (std::cout << __X__ <<std::endl)
#define OUTT(__X__, __Y__) (std::cout << __X__ << __Y__ << std::endl)
// #define OUT_INFO_P(*__X__) (std::cout << __X__ <<std::endl)
//
// template<typename print_type>
// void multi_print(print_type* item_pointer)
// {
//   for(int i =0; i < len(item_pointer-1); ++i)
//   {
//     std::cout<< *item_pointer[i] << " ";
//   }
//   std::cout <<"\n";
// }
