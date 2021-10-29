class bresenham:
    def __init__(self, start, end):
        #self.start = list(start)
        #self.end = list(end)
        self.start = start
        self.end = end
        self.path = []
        self.flag = 0

        if (start[0] == end[0]) and (start[1] == end[1]):
            return 

        # Ensure the Case: dx>dy, that is, k<1.
        self.steep = abs(self.end[1]-self.start[1]) > abs(self.end[0]-self.start[0])
        if self.steep:
            self.start = self.swap(self.start[0],self.start[1])
            self.end = self.swap(self.end[0],self.end[1])

        if self.start[0] > self.end[0]:
            self.flag = 1
            _x0 = int(self.start[0])
            _x1 = int(self.end[0])
            self.start[0] = _x1
            self.end[0] = _x0

            _y0 = int(self.start[1])
            _y1 = int(self.end[1])
            self.start[1] = _y1
            self.end[1] = _y0

        #after all settings are done, we begin besenham algorithm.
        dx = self.end[0] - self.start[0]
        dy = abs(self.end[1] - self.start[1])
        error = 0
        derr = dy/float(dx)

        ystep = 0
        y = self.start[1]

        if self.start[1] < self.end[1]:
            ystep = 1
        else:
            ystep = -1

        for x in range(self.start[0], self.end[0]+1):
            if self.steep:
                self.path.append((y,x))
            else:
                self.path.append((x,y))

            error += derr

            if error >= 0.5:
                y += ystep
                error -= 1.0

        if self.flag == 1:
            self.path.reverse()

    def swap(self,n1,n2):
        return [n2,n1]