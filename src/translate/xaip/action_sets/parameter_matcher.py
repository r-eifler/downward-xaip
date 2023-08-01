class ParamMatcher:

    def __init__(self):
        self.matcher = {}
        self.num_params = -1
  
    def addAction(self,a):
        assert self.num_params  == -1 or len(a.params) == self.num_params, "Num params: "  + str(self.num_params) + " added params: " + str(a.params)
        self.num_params = len(a.params)
        current_matcher = self.matcher
        for i in range(len(a.params)):
            param = a.params[i]
            if not param in current_matcher:
                current_matcher[param] = {}
            current_matcher = current_matcher[param]

    def match(self,params):
        assert(self.num_params == len(params))
        current_matcher = self.matcher
        for i in range(len(params)):
            param = params[i]
            if not param in current_matcher:
                if not "*" in current_matcher:
                    return False
                else:
                    current_matcher = current_matcher["*"]
                    continue
            current_matcher = current_matcher[param]
        return True

    def __repr__(self):
        return str(self.matcher)
        
