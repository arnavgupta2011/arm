import copy,math,numpy
import numdifftools as nd 

class Derivative:

    def df_dx(self,f,factor,var = []):
        x = var[0]
        y = var[1]
        dx = factor*x+factor
        out = (f(x+dx,y) - f(x-dx,y))/(2*dx)
        return out
        
    def df_dy(self,f,factor,var = []):
        x = var[0]
        y = var[1]
        dy = factor*y+factor
        out = (f(x,y+dy) - f(x,y-dy))/(2*dy)
        return out

    def d2f_dx2(self,f,factor,var = []):
        x = var[0]
        y = var[1]
        dx = factor*x+factor
        out = (f(x+dx,y) - 2*f(x,y) + f(x-dx,y))/(dx**2)
        return out
        
    def d2f_dy2(self,f,factor,var = []):
        x = var[0]
        y = var[1]
        dy = factor*y+factor
        out = (f(x,y+dy) - 2*f(x,y) + f(x,y-dy))/(dy**2)
        return out

    def d2f_dxdy(self,f,factor,var = []):
        x = var[0]
        y = var[1]
        dy = factor*y+factor
        dx = factor*x+factor
        out = (f(x+dx,y+dy)+f(x-dx,y-dy)-f(x+dx,y-dy)-f(x-dx,y+dy))/(4*dx*dy)
        return out
    
class Function:

    def __init__(self, exp):
        self.iterations = 0
        self.expression = exp

    def reset_iterations(self):
        self.iterations = 0

    def calc(self, *args):
        self.iterations += 1
        return self.expression(*args)    

def degree(rad):
	return (rad*180)/math.pi

def grad(f,point = []):
    out = nd.Gradient(f)(point)
    return out

def nth_derivative(f,n0,point = []):
    fnd = nd.Derivative(f,n = n0)
    out = fnd(point)

def hess(f,point = []):
        out = nd.Hessian(f)(point)
        return out

def inv_hess(f,point = []):
        matrix = nd.Hessian(f)(point)
        inverse = numpy.linalg.inv(matrix)
        return inverse

def hooke_jeeves(point, func, dx, e, print_stats=False):
    if not isinstance(func, Function):
        raise ValueError("func parameter has to be of type Function")

    x0 = copy.deepcopy(point)
    xb = copy.deepcopy(point)
    xp = copy.deepcopy(point)
    xn = None
    k=1
    func.reset_iterations()
    print("\nInitial best point(BP) = present point(PP): {0}\n".format(xb))
    while True:
        print("ITR:{0}".format(k))
        if dx <= e:
            break
        xn = hooke_pattern_search(xp,func,dx)
        #xn = _hooke_jeeves_search(xp, func, dx)
        print("So now present point(PP) is : {0}".format(xp))
        print("New Point(NP) after exploratory search at Present point(PP)) is {0}".format(xn))
        f_xn = func.calc(*xn) #point after expatory move
        f_xb = func.calc(*xb) #curr_point
        f_xp = func.calc(*xp)
        print("F_value(Fmin) at NP is {0} and F_value at Current BP is {1}".format(round(f_xn,3),round(f_xb,3)))
        if f_xn < f_xb:
            print("F(NP) < F(BP) so updating our Best point")
            for i in range(0, len(xn)):
                xp[i] = 2*xn[i] - xb[i]
                xb[i] = xn[i] 
            print("Updating our previous Best Point(BP): {0} ".format(xb))
            print("Next Present Point for exploratory search found by pattern search method: {0}".format(xp))
        else:
            print("As F(NewPoint) > F(Newpoint) divide Lambda by 2 and repeat Pattern search")
            dx /= 2
            print("New lambda  = {0}".format(dx))
            xp = copy.deepcopy(xb)
            
        k+=1
        print("\n")
    return xb

def hooke_pattern_search(xp, func, dx):
    x = copy.deepcopy(xp)
    for i in range(0, len(xp)):
        p = func.calc(*x) #f_v at curr_point
        k1 = x[i]
        print("F for x[{1}] = {2} in exploratory search: {0}".format(round(p,4),i,x[i]))
        x[i] += dx
        k2 = x[i]
        n_p = func.calc(*x) #f_v at x+dx
        print("F+ for x[{1}]+dx = {2} in exploratory search: {0}".format(round(n_p,4),i,x[i]))
        x[i] -= 2*dx
        k3 = x[i]
        n_m = func.calc(*x) #f_v at x-dx
        print("F- for x[{1}]-dx = {2} in exploratory search: {0}".format(round(n_m,4),i,x[i]))
        fmin = min(n_p,p,n_m)
        ind = (n_p,p,n_m).index(fmin)
        x_arr =(k2,k1,k3)
        xmin = x_arr[ind]
        print("\nFrom above results Fmin is {0} at X[{1}] = {2}\n".format(fmin,i,xmin))
        x[i] = xmin
    return x
         
        
def _hooke_jeeves_search(xp, func, dx):
    x = copy.deepcopy(xp)
    for i in range(0, len(xp)):
        p = func.calc(*x) #f_v at curr_point
        print("F for x[{1}] = {2} in pattern search: {0}".format(round(p,4),i,x[i]))
        x[i] += dx
        n = func.calc(*x) #f_v at x+dx
        print("F+ for x[{1}]+dx = {2} in pattern search: {0}".format(round(n,4),i,x[i]))
        if n > p:
            x[i] -= 2*dx
            n = func.calc(*x) #f_v at x-dx
            print("F- for x[{1}]-dx = {2} in pattern search: {0}".format(round(n,4),i,x[i]))
            if n > p:
                x[i] += dx
    print("From these data Fmin and Xmin can be found")
    return x

def golden_section_search(func, var1, point = False, e=0.00001, print_stats = False):
    
    if point == False and isinstance(var1, list) and len(var1) == 2:
        [a, b] = var1
    k = 0.5 * (math.sqrt(5) - 1)

    c = b - k * (b - a)
    d = a + k * (b - a)
    fc = func(c)
    fd = func(d)

    while (b - a) > e:
        if fc < fd:
            b = d
            d = c
            c = b - k * (b - a)
            fd = fc
            fc = func(c)
        else:
            a = c
            c = d
            d = a + k * (b - a)
            fc = fd
            fd = func(d)
        if print_stats == True:
            print("a: {0} c: {1} d: {2} b: {3} fc: {4} fd: {5}".format(
                a, c, d, b, fc, fd))
    return (a+b)/2
    
def _unimodal_interval(point, idx, func, h=1):

    if not isinstance(func, Function):
        raise ValueError("func parameter has to be of type Function")

    # if func.dim != 1:
    #   raise AttributeError("Unimodal search doesn't apply to multi dimension functions!")

    l = copy.deepcopy(point)
    l[idx] -= h
    r = copy.deepcopy(point)
    r[idx] += h
    m = copy.deepcopy(point)
    step = 1

    fm = func.calc(*point)
    fl = func.calc(*l)
    fr = func.calc(*r)

    if fm < fr and fm < fl:
        return [l, r]
    elif fm > fr:
        while True:
            l = m
            m = copy.deepcopy(r)
            fm = fr
            step *= 2
            r[idx] = point[idx] + h * step
            fr = func.calc(*r)
            if fm <= fr:
                break
    else:
        while True:
            r = m
            m = copy.deepcopy(l)
            fm = fl
            step *= 2
            l[idx] = point[idx] - h * step
            fl = func.calc(*l)
            if fm <= fl:
                break

    return [l, r]

def distab(a,b):
    a+=0.0001
    b+=0.0001
    out = math.sqrt(a**2 + b**2)
    return out

def squareadd(n,arr = []):
    totalsum = 0 
    for i in range (0,n):
        totalsum += arr[i]**2
    out = math.sqrt(totalsum)
    return out

def bounding_method(function,x_inital, delta):
    k = 0
    array = numpy.zeros(3)
    array[2] = x_inital

    fx = function(x_inital)
    fx_neg = function(x_inital-delta)
    fx_pos = function(x_inital+delta)

    for i in range(50):
        if (fx_neg >= fx and fx >= fx_pos):
            break
        elif (fx_neg <= fx and fx <= fx_pos):
            delta *= -1
            break
        else:
            x_inital = -x_inital
    
    for i in range(50):
        array[0] = array[1]
        array[1] = array[2]
        array[2] = array[2] + (2**i)*delta
        fxk1 = function(array[2])
        fxk = function(array[1])
        if (fxk1 >= fxk):
            break
    return [array[0], array[2]] 
     
PHI = 1.6180339887499
REVERSED_PHI = 1/PHI


def calculate(goal_function, ar = []):
    x = 0
    a = ar[0]
    b = ar[1]
    epsilon = 0.000001
    while (b - a) > epsilon:
        lamda = b - (b - a)*REVERSED_PHI
        mu = a + (b - a)*REVERSED_PHI

        if goal_function(lamda) <= goal_function(mu):
            b = mu
            x = lamda
        else:
            a = lamda
            x = mu

    return x

def probe(search= []):
    if golden_function(point + numpy.array(search)*0.01) > golden_function(point):
        search = numpy.array(search)*-1
        print("reversed")
        print(search)
    return search
          
def go_search(oneDimfnc,point = [],search_dir = []): #remove pop for other applications its just for robotic arm IK solver
    past_point = point
    #search_dir = probe(search_dir)
    golden_range = bounding_method(oneDimfnc,x_inital = 1, delta = 0.001)
    golden_range.sort()
    d = golden_section_search(oneDimfnc,golden_range)
    point = past_point + numpy.array(search_dir)*d
    return [point,d]

def isConjugate(new_d = [],c =[], prev_d =[]):
    if (len(new_d) != 2 or len(prev_d) !=2):
        print("All entries should be in row matrix")
        flag = False
    else:
        flag = True
        
    if flag == True:
        new_d = numpy.transpose(new_d)
        mat = numpy.dot(c,new_d)
        result = numpy.dot(prev_d,mat)
        if abs(result) <= 0.1:
            return True
        else:
            return False    
    else:
        print("Error in input.Please check matrix dimensions")
        return 0
