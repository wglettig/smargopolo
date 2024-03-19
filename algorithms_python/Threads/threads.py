#! /usr/bin/python3

# This example shows how to launch threads from a Python script

import threading 
import time

start = time.perf_counter() #start the stopwatch

#Let's define the Function to execute in the thread:
def do_something():
    print('Sleeping 1 second...')
    time.sleep(1)
    print('Done sleeping.')


#Declare the thread, link it to the function above:
t1 = threading.Thread(target=do_something)
t2 = threading.Thread(target=do_something)

#Start the thread:
t1.start()   #start the first thread, then continues the script
t2.start()   #start the second thread, then continues the script

#join:
t1.join()
t2.join()

finish = time.perf_counter()
print (f'Finished in {round(finish-start,2)} second(s)')


print ('===============================================')
#Alternatively we can use the concurrent.futures module

import concurrent.futures

def do_something_for(seconds):
    print(f'Sleeping {seconds} second(s)...')
    time.sleep(seconds)
    return f'Done Sleeping {seconds} second(s).'


with concurrent.futures.ThreadPoolExecutor() as executor:
    secs = [5, 4, 3, 2, 1]
    results = [executor.submit(do_something_for, sec) for sec in secs]
    
    for f in concurrent.futures.as_completed(results):
        print(f.result())
