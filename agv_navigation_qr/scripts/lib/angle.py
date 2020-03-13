from math import pi, fmod, fabs


def diffAng(target_ang, source_ang):
    a = target_ang - source_ang
    #a = fmod(a + pi, 2*pi) - pi
    #return a+2*pi if a <= -pi else a
    a = fmod(a,2*pi)
    if a <= -pi:
      return a+2*pi
    elif a > pi:
      return a-2*pi
    return a


def calcRoute(from_ang, route_ang, to_ang):
    reverse_route_ang = route_ang + pi
    route_dist = fabs(diffAng(route_ang, from_ang)) + \
        fabs(diffAng(to_ang, route_ang))
    reverse_dist = fabs(diffAng(reverse_route_ang, from_ang)) + \
        fabs(diffAng(to_ang, reverse_route_ang))
    return diffAng(route_ang, from_ang) if route_dist <= reverse_dist else diffAng(reverse_route_ang, from_ang)
