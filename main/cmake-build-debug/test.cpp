//
// Created by Kaeshev Alapati on 9/20/25.
//


#include <iostream>

class IntVector {
public:
    int current_size;
    int max_size;
    int growth_factor = 2;
    int * vector_ptr;

    IntVector(int _size)
    {
        max_size = (int)(size * growth_factor);
        vector_ptr = new int[max_size];
        current_size = _size;
    };


    push_back(int element)
    {
        if (current_size == max_size) {
            max_size = (int) (max_size * growth_factor);
            int *temp_new_vector_ptr = new int[max_size];

            for (int i = 0; i < current_size; i++) {
                temp_new_vector_ptr[i] = vector_ptr[i];
            }
            vector_ptr = temp_new_vector_ptr;
            delete temp_new_vector_ptr[];
        }

        vector_ptr[current_size] = element;
        current_size++;
    }
};

int main() {

}