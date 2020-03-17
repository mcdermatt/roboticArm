from __future__ import print_function, division
from sympy import init_printing
from sympy.abc import c, d, e, f, g, h
from sympy.physics.vector import ReferenceFrame, dot
from sympy import acos
from sympy.printing.pretty.pretty import pretty_print


init_printing(use_latex=None, pretty_print=True,auto_symbols=True)

N = ReferenceFrame('N')

a = c * N.x + d * N.y + e * N.z
b = f * N.x + g * N.y + h * N.z

ans = acos((a.dot(b)/(a.magnitude()*b.magnitude())))

print(ans)
pretty_print(ans)
