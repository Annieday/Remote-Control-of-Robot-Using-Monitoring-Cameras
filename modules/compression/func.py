import numpy as np
import matplotlib.pyplot as plt
import cv2
import scipy.fftpack, scipy.stats
# from common_func import *


Q = np.array([[16, 11, 10, 16, 24,  40,  51,  61 ],
              [12, 12, 14, 19, 26,  58,  60,  55 ],
              [14, 13, 16, 24, 40,  57,  69,  56 ],
              [14, 17, 22, 29, 51,  87,  80,  62 ],
              [18, 22, 37, 56, 68,  109, 103, 77 ],
              [24, 35, 55, 64, 81,  104, 113, 92 ],
              [49, 64, 78, 87, 103, 121, 120, 101],
              [72, 92, 95, 98, 112, 100, 103, 99 ]])

a = 2
def dct2(src):
    return scipy.fftpack.dct(
            scipy.fftpack.dct( src, axis=0, type=a, norm='ortho' ),
            axis=1, type=a, norm='ortho' )

def idct2(src):
    return scipy.fftpack.idct(
            scipy.fftpack.idct( src, axis=0, type=a, norm='ortho'),
            axis=1, type=a,norm='ortho')

def blockDCT(f, blockSize, q):
    size = f.shape
    g = np.zeros(size)
    f = f - 128

    # Do blockSize DCT on image (in-place)
    for i in range(0,size[0],blockSize[0]):
        for j in range(0,size[1],blockSize[1]):
            sub_block = f[i:(i+blockSize[0]),j:(j+blockSize[1])]
            sub_block_size = sub_block.shape
            g[i:(i+blockSize[0]),j:(j+blockSize[1])] = \
                dct2( sub_block ) * (q.astype(float)[:sub_block_size[0], :sub_block_size[1]])
    # g = round(dct2(f)/q.astype(float))
    return g

def blockIDCT(g, blockSize, q):
    size = g.shape
    h = np.zeros(size)

    # Do blockSize IDCT on image (in-place)
    for i in range(0,size[0],blockSize[0]):
        for j in range(0,size[1],blockSize[1]):
            h[i:(i+blockSize[0]),j:(j+blockSize[1])] = \
                idct2( g[i:(i+blockSize[0]),j:(j+blockSize[1])])# * q.astype(float))

    h = h + 128
    return h

def lengthCoding(zigzaged_image):
    length_coded_image = []
    for i in range(len(zigzaged_image)):
        if len(length_coded_image) % 2 == 0:
            length_coded_image.append(zigzaged_image[i])
            current_length = 1
        else:
            if zigzaged_image[i] == length_coded_image[-1]:
                current_length += 1
            else:
                length_coded_image.append(current_length)
                length_coded_image.append(zigzaged_image[i])
                current_length = 1
    length_coded_image.append(current_length)
    # print(len(length_coded_image))
    return length_coded_image

def lengthDecoding(length_coded_image):
    decoded_img = []
    for i in range(len(length_coded_image)):
        if i % 2 == 0:
            value = length_coded_image[i]
        else:
            # print(length_coded_image[i])
            for j in range(length_coded_image[i]):
                decoded_img.append(value)
    # print(len(decoded_img))
    return decoded_img


def blockZigzag(f, blockSize, q):
    size = f.shape
    g = np.array([], dtype=np.uint8)

    # Do blockSize zigzag on image (in-place)
    for i in range(0,size[0],blockSize[0]):
        for j in range(0,size[1],blockSize[1]):
            block_flatten = zig_zag(f[i:(i+blockSize[0]),j:(j+blockSize[1])])
            # if False in (unzig_zag(blockSize, block_flatten) == f[i:(i+blockSize[0]),j:(j+blockSize[1])]):
            #     print("Zig zag problem")
            g = np.append(g, block_flatten)
            # print(block_flatten)
    # if False in (blockUnzigzag(size, g, blockSize, q).astype(np.uint8) == f):
    #     print("Problem with blockZigzag")
    #     print(f)
    #     print(blockUnzigzag(size, g, blockSize, q))
    return g

def blockUnzigzag(size, f, blockSize):
    # size = f.shape
    out = np.zeros(size)
    # g = np.array([])

    # Do blockSize unzigzag on image (in-place)
    block_start_point = 0
    for i in range(0,size[0],blockSize[0]):
        for j in range(0,size[1],blockSize[1]):
            sub_block_height = min(size[0] - i, blockSize[0])
            sub_block_width = min(size[1] - j, blockSize[1])
            sub_block_size = (sub_block_height, sub_block_width)
            # if sub_block_size != (8, 8):
            #     print(sub_block_size)
            decoded_block = unzig_zag(sub_block_size, f[block_start_point: block_start_point+sub_block_height*sub_block_width])
            out[i:(i+sub_block_size[0]),j:(j+sub_block_size[1])] = decoded_block
            block_start_point += (sub_block_height*sub_block_width)
            # print(block_flatten)
    # g = round(dct2(f)/q.astype(float))
    return out

def unzig_zag(size, zigzagged_img):
    # size = src.shape
    out = np.zeros(size)
    # out = list()
    row, col = 0, 0
    change = -1

    i = 0
    while row in range(size[0]) and col in range(size[1]):
        # raw_input(str(row)+" "+str(col))
        # out.append(src[row, col])
        out[row, col] = zigzagged_img[i]
        i += 1

        # handle bounding cases
        if (row==0 or row==size[0]-1) and col!=size[1]:
            col += 1
            change = change*(-1)
            # raw_input(str(row)+" "+str(col))
            # out.append(src[row, col])
            out[row, col] = zigzagged_img[i]
            i += 1

        elif (col==0 or col==size[1]-1) and row!=size[0]:
            row += 1
            change = change*(-1)
            # raw_input(str(row)+" "+str(col))
            # out.append(src[row, col])
            out[row, col] = zigzagged_img[i]
            i += 1

        # handle general cases
        row += change
        col -= change

    # out = np.array(out)
    return out

def zig_zag(src):
    size = src.shape
    out = list()
    row, col = 0, 0
    change = -1

    while row in range(size[0]) and col in range(size[1]):
        # raw_input(str(row)+" "+str(col))
        out.append(src[row, col])

        # handle bounding cases
        if (row==0 or row==size[0]-1) and col!=size[1]:
            col += 1
            change = change*(-1)
            # raw_input(str(row)+" "+str(col))
            out.append(src[row, col])

        elif (col==0 or col==size[1]-1) and row!=size[0]:
            row += 1
            change = change*(-1)
            # raw_input(str(row)+" "+str(col))
            out.append(src[row, col])

        # handle general cases
        row += change
        col -= change

    out = np.array(out)
    return out


def entropy(src):
    """
    This function return the entropy of src

    param: src
    param type: numpy.dtarray
    return: entropy
    return type: float
    """
    # initialization
    entropy = 0
    unique, counts = np.unique(src, return_counts=True)
    total_count = float(sum(counts)) # could the total number of

    # calcutate the entropy
    for i in range(len(unique)):
        P = counts[i]/total_count # compute the probability for a unique pixel value
        entropy += -P*np.log2(P)

    return entropy

def blockEntropy(g, blockSize):
    size = g.shape
    E, block_num = 0, 0
    for i in range(0,size[0],blockSize[0]):
        for j in range(0,size[1],blockSize[1]):
            array_1d = zig_zag( g[i:(i+blockSize[0]),j:(j+blockSize[1])] )
            E += entropy(array_1d)
            block_num += 1
    h = E/block_num
    return h

def compressionRatio(f, q):
    block_size = q.shape
    # encode
    encoded_img = blockDCT(f.astype(np.float64), block_size, q)

    """
    Calculate the possible compression ratio
    """
    encoded_entropy = blockEntropy(encoded_img, block_size)
    c = 8 / encoded_entropy
    return c

def PSNR(src, rec, max_value):
    MSE = np.sum((src - rec)**2) / (src.shape[0] * src.shape[1])
    out = 10*np.log(max_value**2/MSE)
    return out






def normalize(src):
    max_value = src.max()
    min_value = src.min()

    src = src - min_value
    src = src.astype(np.float64)
    src /= (max_value - min_value)
    src *= 255.0
    src =  src.astype(np.uint8)
    return src

def normalize_cutoff_lower(src):
    size = src.shape
    return (src > np.zeros(size)) * np.ones(size) * src

def normalize_cutoff_higher(src):
    size = src.shape
    mask = np.ones(size)*255
    not_higher_255 = (src < mask) * np.ones(size) * src
    return not_higher_255 + (src >= mask) * np.ones(size) * 255

def normalize_cutoff_higher_and_lower(src):
    src = normalize_cutoff_higher(src)
    src = normalize_cutoff_lower(src)
    return src

def compute_MSE(src, rec):
    return np.sum((src.astype("float64") - rec.astype("float64"))**2) / float(src.shape[0] * src.shape[1])

def compute_MDE(src, rec):
    return np.max(abs(src.astype("float64") - rec.astype("float64")))

def createQ(L):                 # Create Q
    Q = np.zeros((8,8))
    Q[:L,:L] = np.ones((L,L))
    return Q






if __name__ == "__main__":
    """
    Main function
    """
    img = cv2.imread("boy.tif", cv2.IMREAD_GRAYSCALE)
    # cv2.imshow("boy.tif", img) # show input image
    # cv2.waitKey()

    # encode
    encoded_img = blockDCT(img.astype(np.float64),Q.shape,Q)
    # cv2.imshow("encoded boy.tif", encoded_img.astype("int8")) # show encoded image
    # cv2.waitKey()

    # decode
    decoded_img = blockIDCT(encoded_img, Q.shape, Q)

    # compute compression ration
    ratio = compressionRatio(img, Q)
    print("The possible compression ratio is", ratio)

    # compute PSNR
    psnr = PSNR(img, decoded_img, 255)
    print("The PSNR is", psnr)

    # show decoded image
    # cv2.imshow("decoded boy.tif", decoded_img.astype("uint8")) # show decoded image
    # cv2.waitKey()

    cv2.destroyAllWindows()

    # showing all the results at once
    plt.subplot(1,3,1)
    plt.imshow(img,'gray')
    plt.title("input image")
    plt.xticks([]),plt.yticks([])

    plt.subplot(1,3,2)
    plt.imshow(encoded_img,'gray')
    plt.title("encoded boy.tif")
    plt.xlabel('The possible compression ratio: '+str(ratio))
    plt.xticks([]),plt.yticks([])

    plt.subplot(1,3,3)
    plt.imshow(decoded_img.astype("uint8"),'gray')
    plt.title("decoded boy.tif")
    plt.xlabel("The PSNR: "+str(psnr))
    plt.xticks([]),plt.yticks([])

    plt.show()

    # cv2.waitKey()
