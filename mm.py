# %%
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#import jupyter
import matplotlib.pyplot as plt
# %%
# Se definen las condiciones de las entradas y salidas del sistema,
# en este caso las entradas son la calidad y el servicio, y la salida será la propina
quality = ctrl.Antecedent(np.arange(0, 11, 1), 'quality')
service = ctrl.Antecedent(np.arange(0, 11, 1), 'service')
tip = ctrl.Consequent(np.arange(0, 26, 1), 'tip')

# Generación automática de las funciones de pertenencia de la población
quality.automf(7)
service.automf(5)

# Construcción de las funciones de pertenencia  de manera personalzada
tip['low'] = fuzz.trimf(tip.universe, [0, 0, 13])
tip['medium'] = fuzz.trimf(tip.universe, [0, 13, 25])
tip['high'] = fuzz.trimf(tip.universe, [13, 25, 25])
# Para conocer más funciones de pertenencia visitar: http://pythonhosted.org/scikit-fuzzy/api/skfuzzy.membership.html
# Se grafican las funciones de transferencia
# %%
quality['average'].view()
plt.show()
# %%
service.view()
plt.show()
# %%
tip.view()
plt.show()
# %%
# Se crean las reglas de inferencia para el problema
rule1 = ctrl.Rule(quality['poor'] | service['poor'], tip['low'])
rule2 = ctrl.Rule(service['average'], tip['medium'])
rule3 = ctrl.Rule(service['good'] & quality['good'], tip['high'])
rule1.view()
plt.show()
# %%
# Se crea el contorlador del sistema
tipping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
# Se realiza una simulación dle controlador par auna cituación en específico
tipping = ctrl.ControlSystemSimulation(tipping_ctrl)
# Se dan valores a las entradas del sistema
tipping.input['quality'] = 0
tipping.input['service'] = 0

# Se procesan los datos y se obtiene el resultado
tipping.compute()
print(tipping.output['tip'])
tip.view(sim=tipping)
plt.show()