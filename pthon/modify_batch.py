__author__ = 'houwenbo'
import os
import cPickle


def cvt_typecode(label):
    typecode = '0'
    if label == 1:
        typecode = '60101'
    elif label == 2:
        typecode = '60102'
    elif label == 3:
        typecode = '60103'
    elif label == 4:
        typecode = '60104'
    elif label == 5:
        typecode = '60105'
    elif label == 6:
        typecode = '60106'
    elif label == 7:
        typecode = '60107'
    elif label == 8:
        typecode = '60108'
    elif label == 9:
        typecode = '60109'
    elif label == 10:
        typecode = '60110'
    elif label == 11:
        typecode = '60111'
    elif label == 12:
        typecode = '60117'
    elif label == 13:
        typecode = '60118'
    elif label == 14:
        typecode = '60201'
    elif label == 15:
        typecode = '60202'
    elif label == 16:
        typecode = '60203'
    elif label == 17:
        typecode = '60204'
    return typecode

def get_typecode(label):
    typecode = 0
    if label == '60101':
        typecode = 1
    elif label == '60102':
        typecode = 2
    elif label == '60103':
        typecode = 3
    elif label == '60104':
        typecode = 4
    elif label == '60105':
        typecode = 5
    elif label == '60106':
        typecode = 6
    elif label == '60107':
        typecode = 7
    elif label == '60108':
        typecode = 8
    elif label == '60109':
        typecode = 9
    elif label == '60110':
        typecode = 10
    elif label == '60111':
        typecode = 11
    elif label == '60117':
        typecode = 12
    elif label == '60118':
        typecode = 13
    elif label == '60201':
        typecode = 14
    elif label == '60202':
        typecode = 15
    elif label == '60203':
        typecode = 16
    elif label == '60204':
        typecode = 17
    return typecode

if __name__=='__main__':
    file_name = 'D:/60101/60101_modify.txt'
    #file_name = 'D:/all_train/all_train.txt'
    batch_path = 'D:/batchs_train/'
    suffix = '1'

    fp = open(file_name, 'r')
    d = {}
    while True:
        line = fp.readline()
        if line:
            pos = line.find('_')
            typecode = line[:pos]
            #line = line[pos+1:]
            #pos = line.find('_')
            if pos > 0:
                name = line[pos+1:-5] + ".jpg"
                d[name] = typecode
        else:
            break
    fp.close()

    cnt = 0
    batch_names = os.listdir(batch_path)
    for bt_name in batch_names:
        if bt_name == 'batches.meta':
            continue
        if bt_name[-1] != suffix:
            continue
        print bt_name
        datafile = open(batch_path + bt_name)
        dataDic = cPickle.load(datafile)
        datafile.close()
        filenames = dataDic['filenames']
        lables = dataDic['labels']
        for i in range(0, len(dataDic['labels'])):
            filename = filenames[i]
            if d.has_key(filename):
                print filenames[i]
                lables[i] = get_typecode(d[filenames[i]])
                print dataDic['labels'][i]
                cnt = cnt+1
        savefile = open(batch_path + bt_name, 'w')
        cPickle.dump(dataDic, savefile)
        savefile.close()
    print cnt