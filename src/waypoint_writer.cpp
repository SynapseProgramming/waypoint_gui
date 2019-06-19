// using ofstream constructors.
#include <iostream>
#include <fstream>
#include <vector>

/*
std::ofstream outfile("test.yaml");

outfile << "hello world!" << std::endl;

outfile.close();
*/
int main(){

int state=0;
int input=0;
double vect_input=0;
bool stop=false;
std::vector<double> store;
std::ofstream outfile("waypoint_test.yaml");

while(stop!=true){
std::cout<<"Please enter (1) to enter a number to add to the array"<<std::endl;
std::cout<<"Please enter (2) to export the array to a text file, and exit the program"<<std::endl;
std::cin>>input;
if(input==1){state=1;}
else if(input==2){state=2;}

switch(state){
case 1:
std::cout<<"Please press and enter a number!"<<std::endl;
std::cin>>vect_input;
store.push_back(vect_input);
std::cout<<"no. of elements in vector: "<<store.size()<<std::endl;
break;

case 2:
outfile << "x: [";
for(unsigned int i=0; i<store.size();i++){
//std::cout<<"The ["<<i+1<<"] element is: "<<store[i]<<std::endl;
//i is at the last element of the vector
if(i==(store.size()-1)){outfile<<store[i]<<"]\n";}
else{outfile<<store[i]<<",";}
}
outfile<<"this statement is to show if new line was executed";
outfile.close();
stop=true;
break;
}//bracket of switch statement

}

return 0;}
