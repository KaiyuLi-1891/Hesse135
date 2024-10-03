from xfoil import XFoil
from xfoil.test import naca0012
import math
def calculateClCd1(AoA):
    AoA = round(AoA)
    print(AoA)
    cl, cd, _, _ = xf.a(AoA)

    if not math.isnan(cl):
        pass
    else:
        print("Cl data missing:", AoA)

        # Search for valid cl and cd values within a range of 20 increments
        for i in range(20):
            cl_pos = xf.a(AoA + i + 1)[0]  # cl at AoA + i + 1
            cl_neg = xf.a(AoA - i - 1)[0]  # cl at AoA - i - 1

            if not math.isnan(cl_pos):  # If positive AoA has valid cl
                cl = cl_pos
                break
            elif not math.isnan(cl_neg):  # If negative AoA has valid cl
                cl = cl_neg
                break

    if not math.isnan(cd):
        pass
    else:
        print("Cd data missing:", AoA)

        # Search for valid cl and cd values within a range of 20 increments
        for i in range(20):
            cd_pos = xf.a(AoA + i + 1)[1]  # cl at AoA + i + 1
            cd_neg = xf.a(AoA - i - 1)[1]  # cl at AoA - i - 1

            if not math.isnan(cd_pos):  # If positive AoA has valid cl
                cd = cd_pos
                break
            elif not math.isnan(cd_neg):  # If negative AoA has valid cl
                cd = cd_neg
                break

    return cl, cd


def calculateClCd(AoA):
    AoA = round(AoA)
    print(f"Checking AoA: {AoA}")
    cl, cd, _, _ = xf.a(AoA)

    # Handle missing Cl values
    if math.isnan(cl):
        print(f"Cl data missing at AoA: {AoA}")
        # Search for valid cl within a range of 20 increments
        for i in range(20):
            cl_pos = xf.a(AoA + i + 1)[0]  # cl at AoA + i + 1
            cl_neg = xf.a(AoA - i - 1)[0]  # cl at AoA - i - 1
            print(f"Checking Cl at AoA + {i + 1}: {cl_pos}, AoA - {i + 1}: {cl_neg}")

            if not math.isnan(cl_pos):  # If positive AoA has valid cl
                cl = cl_pos
                print(f"Replaced Cl with value from AoA + {i + 1}: {cl}")
                break
            elif not math.isnan(cl_neg):  # If negative AoA has valid cl
                cl = cl_neg
                print(f"Replaced Cl with value from AoA - {i + 1}: {cl}")
                break

    # Handle missing Cd values
    if math.isnan(cd):
        print(f"Cd data missing at AoA: {AoA}")
        # Search for valid cd within a range of 20 increments
        for i in range(20):
            cd_pos = xf.a(AoA + i + 1)[1]  # cd at AoA + i + 1
            cd_neg = xf.a(AoA - i - 1)[1]  # cd at AoA - i - 1
            print(f"Checking Cd at AoA + {i + 1}: {cd_pos}, AoA - {i + 1}: {cd_neg}")

            if not math.isnan(cd_pos):  # If positive AoA has valid cd
                cd = cd_pos
                print(f"Replaced Cd with value from AoA + {i + 1}: {cd}")
                break
            elif not math.isnan(cd_neg):  # If negative AoA has valid cd
                cd = cd_neg
                print(f"Replaced Cd with value from AoA - {i + 1}: {cd}")
                break

    return cl, cd


if __name__ == '__main__':
    cl_list = []
    cd_list = []
    xf = XFoil()
    xf.airfoil = naca0012
    xf.Re = 1e6
    xf.mat_iter = 40
    for i in range(30):
        cl,cd = calculateClCd(i)
        # cl,cd,_,_ = xf.a(i)
        cl_list.append(cl)
        cd_list.append(cd)
        # print("Angle:",i,"Cl:",cl,"Cd",cd)

    # print(cl_list)
    # print(" ...")
    # print(cd_list)
    for i in range(30):
        print("Angle:",i,"Cl:",cl_list[i],"Cd:",cd_list[i])