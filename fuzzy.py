import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import csv
import pprint as pp
from sklearn.metrics import precision_score, recall_score, f1_score, accuracy_score


class Fuzzy():
    def __init__(self):
        self.simulador : ctrl.ControlSystemSimulation = None

    def training_fuzzy(self, file_text) -> None:
        pulse = ctrl.Antecedent(np.arange(0, 201, 1), 'Pulse')
        qPA = ctrl.Antecedent(np.arange(-10, 11, 1), 'qPA')
        resp = ctrl.Antecedent(np.arange(0, 23, 1), 'Resp')
        classification = ctrl.Consequent(np.arange(1, 5, 1), 'Classification')

        pulse.automf(3, names=['Baixo','Medio','Alto'])  # Gere 3 funções de associação automaticamente para 'Pulse'
        qPA.automf(3, names=['Baixo','Medio','Alto'])    # Gere 3 funções de associação automaticamente para 'qPA'
        resp.automf(3, names=['Baixo','Medio','Alto'])   # Gere 3 funções de associação automaticamente para 'Resp'
        classification.automf(4, names=['Critico','Instavel','Potencialmente Estavel', 'Estavel'])  # Gere 3 funções de associação automaticamente para 'Pulse'
        sample_list = []

        with open (file_text) as inputfile:
            reader = csv.DictReader(inputfile)
            for row in reader:
                q1 = float(row['qPA'])
                p1 = float(row['pulso'])
                r1 = float(row['resp'])
                c1 = int(row['classe'])

                q1_baixo = fuzz.interp_membership(qPA.universe, qPA['Baixo'].mf, q1)
                q1_medio = fuzz.interp_membership(qPA.universe, qPA['Medio'].mf, q1)
                q1_alto = fuzz.interp_membership(qPA.universe, qPA['Alto'].mf, q1)
                q1_maior = max(q1_baixo, q1_medio, q1_alto)
                q1_max_label = 'Baixo' if q1_maior == q1_baixo else 'Medio' if q1_maior == q1_medio else 'Alto' if q1_maior == q1_alto else ' '

                p1_baixo = fuzz.interp_membership(pulse.universe, pulse['Baixo'].mf, p1)
                p1_medio = fuzz.interp_membership(pulse.universe, pulse['Medio'].mf, p1)
                p1_alto = fuzz.interp_membership(pulse.universe, pulse['Alto'].mf, p1)
                p1_maior = max(p1_baixo, p1_medio, p1_alto)
                p1_max_label = 'Baixo' if p1_maior == p1_baixo else 'Medio' if p1_maior == p1_medio else 'Alto' if p1_maior == p1_alto else ' '

                r1_baixo = fuzz.interp_membership(resp.universe, resp['Baixo'].mf, r1)
                r1_medio = fuzz.interp_membership(resp.universe, resp['Medio'].mf, r1)
                r1_alto = fuzz.interp_membership(resp.universe, resp['Alto'].mf, r1)
                r1_maior = max(r1_baixo, r1_medio, r1_alto)
                r1_max_label = 'Baixo' if r1_maior == r1_baixo else 'Medio' if r1_maior == r1_medio else 'Alto' if r1_maior == r1_alto else ' '

                c1_max_label = 'Critico' if c1 == 1 else 'Instavel' if c1 == 2 else 'Potencialmente Estavel' if c1 == 3 else 'Estavel' if c1 == 4 else ' '

                sample = ((q1_maior, q1_max_label),(p1_maior, p1_max_label),(r1_maior, r1_max_label),(c1, c1_max_label))
                sample_list.append(sample)

        all_groups = {}

        for i in sample_list:
            chave = (i[0][1], i[1][1], i[2][1])
            if chave not in all_groups:
                all_groups[chave] = []
            all_groups[chave].append(i)

        categoria_classe = {}

        for categoria, list_tuplonas in all_groups.items():
            maior = 0
            label = ''
            for i in list_tuplonas:
                ativacao = min(i[0][0], i[1][0], i[2][0])
                if ativacao > maior:
                    maior = ativacao
                    label = i[3][1]

            categoria_classe[categoria] = label

        rules_list = []

        for chave, label in categoria_classe.items():
            rule = ctrl.Rule(qPA[chave[0]] & pulse[chave[1]] & resp[chave[2]], classification[label])
            rules_list.append(rule)

        sistema_controle = ctrl.ControlSystem(rules_list)
        self.simulador = ctrl.ControlSystemSimulation(sistema_controle)


    def predict_fuzzy(self, pulse, qPA, resp) -> int:
        self.simulador.input['Pulse'] = pulse
        self.simulador.input['qPA'] = qPA 
        self.simulador.input['Resp'] = resp

        self.simulador.compute()
        resultado_previsto = int(round(self.simulador.output['Classification']))

        return resultado_previsto

    
    def test_dataset_fuzzy(self, file_txt) -> None:

        Y_true = []
        Y_pred = []

        with open (file_txt, 'r') as inputfile:
            reader = csv.DictReader(inputfile)
            for row in reader:
                resultado_previsto = self.predict_fuzzy(float(row['pulso']), float(row['qPA']), float(row['resp']))
                resultado_real = int(row['classe'])
                
                Y_pred.append(resultado_previsto)
                Y_true.append(resultado_real)

        precisao = precision_score(Y_true, Y_pred, average='weighted')
        recall = recall_score(Y_true, Y_pred, average='weighted')
        fmeasure = f1_score(Y_true, Y_pred, average='weighted')
        acuracia = accuracy_score(Y_true, Y_pred)

        print(f"Precisão: {precisao}, Recall: {recall}, F-Measure: {fmeasure}, Acurácia: {acuracia}")
