import sys

# package name
pname = sys.argv[1]


# read model names
stllist = []
fstlist = open("daelist.txt","r")
for fname in fstlist:
    stllist.append(fname.replace(".dae","").replace("\n",""))
print stllist
fstlist.close()
          
for m in stllist:
    t = open("TEMPLATE.urdf","r")
    f = open("urdf/"+m+".urdf","w")
    for line in t:
	line = line.replace("${MODEL_ID}",m)
	line = line.replace("${PACKAGE}",pname)
	print >>f, line,
    print "Did\t\t\t\t"+m
    f.close()
    t.close()



