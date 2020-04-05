import marshal
from returnFuncTest import returnFunction

z = returnFunction(0)

marshal_out = open("z.marshal")

marshal.dumps(z.func_code)