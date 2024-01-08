// collisions-wasm.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "../main.cpp"

int main()
{
    Collisions *collisions = new Collisions();
    collisions->addAABB("a", 0, 0, 16, 16);
    collisions->addAABB("a", 0, 16, 16, 16);
    //b2AABB testbox = collisions->b2AABB_ConstructFromCenterSize(-32, 0, 16, 16);
    b2AABB *testbox = collisions->addAABB("b", -32, 0, 16, 16);
    b2Vec2 res = collisions->test(testbox, 32, 0);

    delete collisions;
    std::cout << "x: " << res.x << " y: " << res.y;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
