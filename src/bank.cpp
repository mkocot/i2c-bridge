#include "bank.hpp"

void bank_t::begin()
{
    Wire.setPins(sda, scl);

    Wire.begin();
}