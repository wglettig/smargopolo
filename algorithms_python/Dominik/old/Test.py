#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 26 15:44:58 2021

@author: ros
"""
import time
from datetime import date
from datetime import datetime

today = date.today()
current_day = today.strftime("%Y_%m_%d_")
print(current_day)

now = datetime.now()
current_time = now.strftime("%H:%M:%S")
print(current_time)

print(current_day+current_time)

