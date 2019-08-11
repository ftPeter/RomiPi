#!/usr/bin/env python


class WaveManager():
    # CONSTANTS
    INACTIVE_STATE = -1

    def __init__(self):
        self.current_wave_dict = dict()
        self.assigned_wave_dict = dict()

    def update(self, update_list):
        for name, assigned, current in update_list:
            self.assigned_wave_dict[name] = assigned
            self.current_wave_dict[name] = current

    def join(self, name, assigned_wave):
        self.assigned_wave_dict[name] = assigned_wave
        self.current_wave_dict[name] = -1

    def leave(self, name):
        if name in self.current_wave_dict:
            del self.current_wave_dict[name]

    def is_active(self, name):
        return self.assigned_wave_dict[name] == self.get_current_active_wave()

    def get_current(self, name):
        return self.current_wave_dict[name]

    def get_current_active_wave(self):
        active_set = set()
        for name in self.current_wave_dict:
            active_set.add(self.current_wave_dict[name])
        if active_set:
            return min(active_set)
        else:
            return -1

    def activity_check(self):
        """ check to see if one is still active in the current state """
        for name in self.current_wave_dict:
            names_current_state = self.current_wave_dict[name]
            active_state = self.get_current_active_wave()
            if names_current_state == active_state:
                return False

    def next_state(self, name):
        """ this is not a my favorite solution, but fine for now
            better solution would be to know the names of all
            the active states and advance through them
        """
        if self.current_wave_dict[name] == -1:
            current_wave = self.get_current_active_wave()
            self.update(name, current_wave)

        next_state = self.current_wave_dict[name] + 1
        if next_state > 10:
            next_state = 0
        return next_state

    def get_contents_list(self):
        """ returns the contents of the activity manager
            formatted for broadcast as an update
        """
        contents = list()
        for name in self.assigned_wave_dict:
            assigned_wave = self.assigned_wave_dict[name]
            current_wave = self.current_wave_dict[name]
            contents.append( (name, assigned_wave, current_wave) )
        return contents

    def __str__(self):
        ret_str = ""
        ret_str += "WaveManager: Wave "
        ret_str += str(self.get_current_active_wave())
        ret_str += " Active. "
        for name in self.current_wave_dict:
            ret_str += "(%s,%d), " % (name, self.current_wave_dict[name])
        return ret_str

def test():
    wm = WaveManager()

    wm.update("alpha", 0)
    wm.update("bravo", 1)
    wm.update("charlie", 2)

    print(wm)

    wm.leave("alpha")

    print(wm)

if __name__ == '__main__':
    test()
