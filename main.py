import os
import json
import GA.bayesian_optimization as GenAlg
# os.system("abaqus cae script=TiePH_4steps_op.py")
if __name__ == "__main__":
    with open(r'x_iter.txt','a+',encoding='utf-8') as test:
        test.truncate(0)

    iter_num = 40

    
    iter_to_Insert = 2         # the iteration start to insert after initial condition setup
    data = {'iteration': 1, 'OverRot': 0, 'RotBac': 0, 'iter_num': iter_num, 'iter_to_Insert': iter_to_Insert}
    with open('PnH_aba/PnH/SimCond.json', 'w', encoding='utf-8') as f:
        json.dump(data, f)
    for i in range(1,iter_num + iter_to_Insert):
        # os.system("abaqus cae noGUI=PnH_aba/PnH/TiePH_4steps.py")
        os.system("abaqus cae script=PnH_aba/PnH/TiePH_4steps.py")
        
        if i != iter_num + iter_to_Insert - 1:
            OPRes = GenAlg.RunGO()     # get optimized rotation angle
            data = {'iteration': data['iteration']+1, 'OverRot': OPRes[0], 'RotBac': OPRes[1], 'iter_num': iter_num, 'iter_to_Insert': iter_to_Insert}
            with open('PnH_aba/PnH/SimCond.json', 'w', encoding='utf-8') as f:
                json.dump(data, f)

    # os.system("abaqus cae script=View/ViewResult.py")