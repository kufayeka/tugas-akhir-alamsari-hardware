class thisIsPythonClass:
    def __init__(self, a, b):
        self.a = 3
        self.b = 4

    def runThis(self):
        print("this is a value:", self.a)
        print("this is b value:", self.b)

if __name__ == '__main__':
    try:
        obj = thisIsPythonClass(1, 2)
        obj.runThis()
    except Exception as e:
        print(e)