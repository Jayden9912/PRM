import os
import argparse

class PRMControllerOptions:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description="PRMController options")

        self.parser.add_argument("--k",
                                 type = int,
                                 help = "number of K nearest neighbours",
                                 default = 15)
        
        self.parser.add_argument("--nbrs",
                                 type = int,
                                 help = "numbers of random sampled points",
                                 default = 1000)

        self.parser.add_argument("--env1",
                                 type = int,
                                 help = "range of x",
                                 default = 10)

        self.parser.add_argument("--env2",
                                 type = int,
                                 help = "range of y",
                                 default = 6)
        
        self.parser.add_argument("--env3",
                                 type = int,
                                 help = "number of obstacles",
                                 default = 5)

        self.parser.add_argument("--shortcut",
                                 help = "perform path shortcutting",
                                 action = "store_true")
        
        self.parser.add_argument("--edgePlot",
                                 help = "plot the edges of the PRM",
                                 action = "store_true")
        
        self.parser.add_argument("--pointPlot",
                                 help = "plot the random sampled points",
                                 action = "store_true")
        
        self.parser.add_argument("--solutionPlot",
                                 help = "plot the solution path",
                                 action = "store_true")

    def parse(self):
        self.options = self.parser.parse_args()
        return self.options