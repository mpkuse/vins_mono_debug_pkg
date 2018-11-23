#include <iostream>

#include "../utils/DisjointSet.h"
using namespace mp;
using namespace std;
int main()
{
    std::cout << "hello\n";
    mp::DisjointSetForest<float> ds;
    ds.add_element( 0, 213.1 );
    ds.add_element( 1, 4.44 );
    ds.add_element( 2, -0.4 );
    ds.add_element( 3, -120.4 );
    ds.union_sets( 0, 1);
    std::cout << "element count: " << ds.element_count() << std::endl;
    cout << "set_count: " << ds.set_count() << endl;

    for( int i=0 ; i<4 ; i++ )
        cout << "find_set: " << i << ":" << ds.find_set( i ) << endl;
}
