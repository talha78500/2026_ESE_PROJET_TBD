# 2026_ESE_PROJET_TBD

## Procédure de test de la carte

### Tests d'alimentation

La 1ère étape pour tester le pcb est de souder les solder bridges. Une fois les soudures faites, une alimentation de laboratoire est utilisée pour alimenter la carte. Pour ne pas endommager la carte en cas de court circuit, le courant de l'alimentation est limité à 100mA. On peut ensuite venir tester un à un les différents points de test de la carte. Les différents points de test sont relié au 5v, 3v3, sys et GND. Lors des des tests d'alimentation, des led temoigneront du fonctionnement : 

<div align="center">
<img width="753" height="331" alt="image" src="https://github.com/user-attachments/assets/f3d6cfae-2d5c-4d91-8483-8c7ba309c1f6" />
</div>

| Test point alimenté | LED Temoin | LED allumée(s) en conséquence |Explication |
| :--- | :--- | :--- | :--- |
| 3v3 | Led rouge D1| X |X |
| 5v  | Led verte D2 | Led verte D2 orange D3 et rouge D4 | Le 5v alimente le régulateur 3v3 donc la led D2 s'allume. |
| SYS | Led rouge D4| X |X |

### Test du chargeur

Avant d'utiliser la batterie, les premiers essais du chargeur sont réalisés avec une alimentation 3v8 1A. Il faut programmer le chargeur via I2C: 
- Désactiver le timer interne (REG07, bits 5:4 = 00)
- Configurer la tension de sortie  boostée (REG0A)
- Couper la charge (REG03) (CHG_CONFIG = 0->Bit 4)
- Activer la sortie boostée (REG03) (OTG_CONFIG = 1 ->Bit 5)

Une fois tout cette partie validée, la batterie peut être branchée.

### Test du fuel gauge 




