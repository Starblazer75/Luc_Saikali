from classes import course
from datetime import datetime


def is_valid(section_1, section_2):
    """
    Checks both times to see if they conflict
    :param section_1: Section class
    :param section_2: Section class
    :return: boolean ; True if they dont conflict, False if they conflict
    """
    section_1_days = set(section_1.days)
    section_2_days = set(section_2.days)
    section_1_start, section_1_end = section_1.start_time, section_1.end_time
    section_2_start, section_2_end = section_2.start_time, section_2.end_time
    result = bool(section_1_days.intersection(section_2_days))

    if not result:
        return True

    if section_1_start <= section_2_end and section_1_end >= section_2_start:
        return False
    else:
        return True


def create_truth_table(courses):
    """
    Sub function ; Creates a truth table to be used later for the list of solutions
    :param courses: courses
    :return: None
    """

    for course_1 in range(len(courses) - 1):
        for index_1 in range(len(courses[course_1].sections)):
            table = []
            for course_2 in range(course_1 + 1, len(courses)):
                sub_table = []
                for index_2 in range(len(courses[course_2].sections)):
                    if is_valid(courses[course_1].sections[index_1], courses[course_2].sections[index_2]):
                        sub_table.append(index_2)
                table.append(set(sub_table[:]))
            courses[course_1].sections[index_1].create_truth(table[:])

    i = 0
    for section in courses[0].sections:
        section.truth_value.section_names.append(i)
        i += 1


def is_failed(section):
    """
    Sub function ; Checks if the solution state is failed
    :param section: looks at the in-progress solution
    :return: boolean
    """
    for i in section:
        if len(i) < 1:
            return False
    return True


def intersect(section_1, section_2):
    """
    Sub function ; Intersects one course with the other
    :param section_1: Section
    :param section_2: Section
    :return: None
    """
    new_section = []
    for i in range(len(section_1) - 1):
        new_section.append(section_1[i + 1].intersection(section_2[i]))

    return new_section


def iterate(lists, courses, sol):
    """
    Main function which gets all the solutions
    :param lists: structure of info
    :param courses: courses
    :param sol: solution list
    :return: None
    """
    sections = lists.sections
    length = len(sections)
    names = lists.section_names
    if length == 1:
        for i in sections[0]:
            new_names = names[:]
            new_names.append(i)
            sol.append(new_names)
    else:
        for i in sections[0]:
            lists.sections = intersect(sections, courses[len(courses) - length].sections[i].truth_value.sections)
            status = is_failed(lists.sections)

            if status is True:
                lists.section_names = names
                lists.section_names.append(i)
                iterate(lists, courses, sol)
                del names[-1]


def quick_sort(days, pointer):
    if len(pointer) <= 1:
        return pointer
    else:
        pivot = pointer[0]
        pivot_start_time = pivot.start_time
        pivot_start_time = pivot_start_time.split(":")
        pivot_start_time[0] = int(pivot_start_time[0])
        pivot_start_time[1] = int(pivot_start_time[1])
        pivot_start_time = (pivot_start_time[0] * 100) + pivot_start_time[1]

        left = []
        right = []
        for x in pointer[1::]:
            x_start_time = x.start_time
            x_start_time = x_start_time.split(":")
            x_start_time[0] = int(x_start_time[0])
            x_start_time[1] = int(x_start_time[1])
            x_start_time = (x_start_time[0] * 100) + x_start_time[1]

            if x_start_time <= pivot_start_time:
                left.append(x)

        for x in pointer[1::]:
            x_start_time = x.start_time
            x_start_time = x.start_time
            x_start_time = x_start_time.split(":")
            x_start_time[0] = int(x_start_time[0])
            x_start_time[1] = int(x_start_time[1])
            x_start_time = (x_start_time[0] * 100) + x_start_time[1]

            if x_start_time > pivot_start_time:
                right.append(x)

        return quick_sort(days, left) + [pivot] + quick_sort(days, right)


def organize(days):
    if days["Monday"]:
        days["Monday"] = quick_sort(days, days["Monday"][:])

    if days["Tuesday"]:
        days["Tuesday"] = quick_sort(days, days["Tuesday"][:])

    if days["Wednesday"]:
        days["Wednesday"] = quick_sort(days, days["Wednesday"][:])

    if days["Thursday"]:
        days["Thursday"] = quick_sort(days, days["Thursday"][:])

    if days["Friday"]:
        days["Friday"] = quick_sort(days, days["Friday"][:])

    if days["Saturday"]:
        days["Saturday"] = quick_sort(days, days["Saturday"][:])

    if days["Sunday"]:
        days["Sunday"] = quick_sort(days, days["Sunday"][:])


def points(days, day, summation):
    for i in range(len(days[day]) - 1):
        j = i + 1

        start_time_j = days[day][j].start_time
        start_time_j = start_time_j.split(":")
        start_time_j[0] = int(start_time_j[0]) * 100
        start_time_j = start_time_j[0] + int(start_time_j[1])

        start_time_i = days[day][i].start_time
        start_time_i = start_time_i.split(":")
        start_time_i[0] = int(start_time_i[0]) * 100
        start_time_i = start_time_i[0] + int(start_time_i[1])

        division = start_time_j - start_time_i

        summation += division

    return summation


def organize_schedule(courses, sol):
    organize_list = []
    for solution in sol:
        days = {"Monday": [],
                "Tuesday": [],
                "Wednesday": [],
                "Thursday": [],
                "Friday": [],
                "Saturday": [],
                "Sunday": []}
        i = 0
        for sub_solution in solution:
            course = courses[i].sections[sub_solution]

            for day in course.days:
                days[day].append(course)
            i += 1

        organize(days)

        summation = 0
        if len(days["Monday"]) > 1:
            summation = points(days, "Monday", summation)

        if len(days["Tuesday"]) > 1:
            summation = points(days, "Tuesday", summation)

        if len(days["Wednesday"]) > 1:
            summation = points(days, "Wednesday", summation)

        if len(days["Thursday"]) > 1:
            summation = points(days, "Thursday", summation)

        if len(days["Friday"]) > 1:
            summation = points(days, "Friday", summation)

        if len(days["Saturday"]) > 1:
            summation = points(days, "Saturday", summation)

        if len(days["Sunday"]) > 1:
            summation = points(days, "Sunday", summation)

        organize_list.append([days, summation])

    return organize_list


def create_schedule(courses):
    """
    Main function which creates the schedule
    :param courses: courses
    :return: Returns a list of all the solutions. Each solution is a list of the index of each section in each
                respective course
    [0, 0, 1, 0, 1]
    Course 0: Section 0
    Course 1: Section 0
    Course 2: Section 1
    Course 3: Section 0
    Course 4: Section 1

    The course lists all the courses which is an Object

    Inside the course object there is a sub_course object (The section)
    To access the info of Course 0 section 0: courses[0].sections[0]

    There you will see the info for the section like the times
    """
    create_truth_table(courses)

    sol = []

    for i in courses[0].sections:
        iterate(i.truth_value, courses, sol)

    return sol


def main():
    """
    Doesnt mean much to you, mainly my way of inserting info manually to test functions
    Good Idea to use this data to test it in the entire program as these are actual times from Fall 2023

    To use this, all you have to do is create each Class Object and add sections using the method
    After you've made all the classes, put them all in a list
    Then you can use create_schedule(courses) where courses is the list of all courses

    It will return a list of all the solutions which you can use. Solutions are explained in the create_schedule method
    :return: None
    """

    CS1300 = course("CS1300")
    CS1300.add_section(72141, ["Monday", "Friday"], "11:30", "12:45", "name", 123)
    CS1300.add_section(74299, ["Monday", "Wednesday"], "14:00", "15:15", "name", 123)
    CS1300.add_section(74300, ["Tuesday", "Thursday"], "10:00", "11:15", "name", 123)
    CS1300.add_section(74301, ["Tuesday", "Thursday"], "13:00", "14:15", "name", 123)

    CS1400 = course("CS1400")
    CS1400.add_section(72142, ["Tuesday", "Thursday"], "13:00", "14:50", "name", 123)
    CS1400.add_section(74303, ["Monday", "Wednesday", "Friday"], "10:45", "11:50", "name", 123)
    CS1400.add_section(74304, ["Monday", "Wednesday"], "15:00", "16:50", "name", 123)
    CS1400.add_section(74305, ["Monday", "Wednesday"], "13:00", "14:50", "name", 123)

    CS2400 = course("CS2400")
    CS2400.add_section(72143, ["Tuesday", "Thursday"], "10:00", "11:50", "name", 123)
    CS2400.add_section(74307, ["Monday", "Wednesday", "Friday"], "9:30", "10:35", "name", 123)
    CS2400.add_section(74308, ["Monday", "Wednesday"], "15:00", "16:50", "name", 123)

    CS3110 = course("CS3110")
    CS3110.add_section(72144, ["Monday", "Wednesday"], "11:30", "12:45", "name", 123)
    CS3110.add_section(74328, ["Tuesday", "Thursday"], "10:00", "11:15", "name", 123)
    CS3110.add_section(74329, ["Tuesday", "Thursday"], "14:30", "15:45", "name", 123)

    CS3310 = course("CS3310")
    CS3310.add_section(72145, ["Tuesday", "Thursday"], "13:00", "14:15", "name", 123)
    CS3310.add_section(74330, ["Monday", "Wednesday"], "14:30", "15:45", "name", 123)
    CS3310.add_section(74331, ["Monday", "Wednesday"], "13:00", "14:15", "name", 123)
    CS3310.add_section(75889, ["Wednesday", "Friday"], "13:00", "14:15", "name", 123)

    courses = [CS1300, CS1400, CS2400, CS3110, CS3310]

    sol = create_schedule(courses)
    organize_list = organize_schedule(courses, sol)
    organize_list = sorted(organize_list, key=lambda x: x[1])

    print(organize_list)


if __name__ == '__main__':
    main()
