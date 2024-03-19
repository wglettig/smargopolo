#include <iostream>
using namespace std;

struct Node { 
    int data; 
    struct Node *next; 
}; 

struct Node* head = NULL;   

void insert(int new_data) { 
    struct Node* new_node = (struct Node*) malloc(sizeof(struct Node)); 
    new_node->data = new_data; 
    new_node->next = head; 
    head = new_node; 
} 

void display() { 
    struct Node* ptr;
    ptr = head;
    while (ptr != NULL) { 
       cout<< ptr->data <<" "; 
       ptr = ptr->next; 
    } 
} 

void remove_first(){
    struct Node* ptr;
    ptr = head;
    head = head->next;
    delete ptr;
}

void remove_last() {
    struct Node *temp1, *temp2;
    if (head == NULL)
         cout << "The list is empty!" << endl;
    else
    {
        temp1 = head;
        while (temp1->next != NULL)
        {
            temp2 = temp1;
            temp1 = temp1->next;
        }
        delete temp1;
        temp2->next = NULL;
    }
}


int main() { 
    insert(3);
    insert(1);
    insert(7);
    insert(2);
    insert(9);
    cout<<"The linked list is populated with: ";
    display(); 
    remove_first();
    cout<<"\nAfter removing the first element : ";
    display(); 
    remove_last();
    cout<<"\nAfter removing the last element  : ";
    display(); 
    cout<<"\n";
    return 0; 
} 
