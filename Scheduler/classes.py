from datetime import datetime


def normalize_time(time_str):
    try:
        # Parse as 24-hour time if possible
        dt = datetime.strptime(time_str, "%H:%M")
    except ValueError:
        # If both fail, return original string
        return time_str

    # Return time in both 24-hour and 12-hour formats for display schedule
    return dt.strftime("%H:%M"), dt.strftime("%I:%M %p")


class course:
    def __init__(self, name):
        # Example name: CS 2400
        self.chosen_section = None
        self.className = name
        self.sections = []
        self.counter = 0

    def __str__(self):
        return self.className

    def add_section(self, section_number, days, start_time, end_time, professor, room):
        # Days will be a string starting with Monday ending on Sunday
        # Example days: Monday, Wednesday
        section = sub_course(self.className, section_number, days, start_time, end_time, professor, room, self)
        self.sections.append(section)

    def delete_section(self, section_number):
        try:
            del self.sections[section_number]
        except KeyError:
            print(f"{section_number} doesn't exist")

    def choose_section(self, section_number):
        self.chosen_section = self.sections[section_number]


class sub_course:
    def __init__(self, course_name, section_number, days, start_time, end_time, professor, room, class_obj):
        self.truth_value = None
        self.course_name = course_name
        self.classSection = section_number
        self.days = days
        self.start_time = start_time
        self.end_time = end_time
        self.professor = professor
        self.room = room
        self.class_obj = class_obj
        self.normalized_start_time, self.display_start_time = normalize_time(start_time)
        self.normalized_end_time, self.display_end_time = normalize_time(end_time)

    def __str__(self):
        return f'{self.classSection} {self.professor} {self.room} {self.start_time} - {self.end_time} {self.days}'

    def create_truth(self, sections):
        self.truth_value = truth(self.course_name, [], sections)

    @property
    def start_time_obj(self):
        format = "%H:%M"
        return datetime.strptime(self.start_time, format).time()

    @property
    def end_time_obj(self):
        format = "%H:%M"
        return datetime.strptime(self.end_time, format).time()


class truth:
    def __init__(self, course_name, section_names, sections):
        self.name = course_name
        self.section_names = section_names
        self.sections = sections
