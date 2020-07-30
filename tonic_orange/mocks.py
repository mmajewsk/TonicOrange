from common.images import read_folder_as_tuples

class TonicMock:
    def __init__(self, images_path, start=0, stop=None):
        self.images_path = images_path
        self.start = start
        self.data = read_folder_as_tuples(self.images_path)
        self.stop = stop
        if self.stop == None:
            self.stop = len(self.data)-1

    def image_now(self):
        for data in self.data[self.start + 1:self.stop]:
            yield data

class DriverMock:
    def __init__(self, tonic):
        self.tmpdumpfilepath = '/home/mwm/repositories/TonicOrange/assets/tmp/arrows.csv'

    def drive(self, current_pose2D, destination2D):
        if current_pose2D is not None:
            with open(self.tmpdumpfilepath, 'a') as f:
                write_line = "{} {} {} {} {} {}".format(*current_pose2D[0], current_pose2D[1], *destination2D[0], current_pose2D[1])
                f.write(write_line+"\n")
        print("Current: {},   Destination:{}".format(current_pose2D, destination2D))
