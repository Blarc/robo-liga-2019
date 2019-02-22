import matplotlib.pyplot as plt

plt.style.use('seaborn-whitegrid')

# fig = plt.figure()
# ax = plt.axes()
# x = 10
# y = 10

# plt.plot(x, y)        # plot x and y using default line style and color
# plt.plot(x, y, 'bo')  # plot x and y using blue circle markers
# plt.plot(y)           # plot y using x as index array 0..N-1
# plt.plot(y, 'r+')     # ditto, but with red plusses

# mp.savefig('foo.png')
# mp.savefig('foo.pdf')

# savefig('foo.png', bbox_inches='tight')

# print(os.path.relpath(data_folder))

x = []
y = []

f = open('..\\..\\pid_data\\pid_data0', 'r')
for i in f:
    s = i.split(',')
    x.append(float(s[0]))
    y.append(float(s[1].strip('\n')))


print(x)
print(y)
plt.plot(x, y)
plt.show()
plt.savefig('..\\..\\plot.png', bbox_inches='tight')
