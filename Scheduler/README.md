# Class Scheduler

This is only the backend because I didn't make the frontend

First, I create all the class objects with all the relevant information

Then, I check to see which schedules work without intersecting

To do this without being extremely inefficient, I created a truth table

Using this truth table, I can give a course a set of other courses which it works with

When trying to create a schedule, I start with the first course's set, and then intersect it with the second course's
set to get the schedule's truth table.

If the set ever goes empty, the schedule is invalid

