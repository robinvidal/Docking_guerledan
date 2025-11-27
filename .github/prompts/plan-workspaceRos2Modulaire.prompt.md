# Plan: Workspace ROS2 modulaire pour mission Docking

**TL;DR**: Construire un workspace testable avec isolation forte entre packages. Démarrer par les messages et utilitaires (fondations), puis implémenter les nœuds en suivant le flux de données (sonar → traitement → tracking → localisation → control). Finir par l'orchestration (mission + bringup). Chaque package doit être compilable, testable et exécutable indépendamment.

## Étapes (ordre de construction recommandé)

**1. Créer `docking_msgs` — Messages communs**
   - Définir `.msg` pour Frame (sonar), Borders (tracking), PoseRelative (localisation), State (mission).
   - Pourquoi en premier: tous les packages en dépendent, évite refactoring massif.
   - Test: génération messages, import Python/C++.

**2. Créer `docking_utils` — Bibliothèque commune**
   - Fonctions réutilisables: conversions polaire↔cartésien, filtres signal, transformations TF, validation géométrie cage.
   - Pourquoi: évite duplication entre traitement/tracking/localisation.
   - Test: unitaires sur chaque fonction (pytest).

**3. Package `sonar` — Acquisition données**
   - Driver Oculus M750d (ou mock pour dev sans matériel).
   - Publie `Frame` sur `/docking/sonar/raw`.
   - Test: mock publisher + vérif format/rate.

**4. Package `traitement` — Filtrage**
   - Souscrit `/sonar/raw`, applique filtres (bruit, contraste), publie `/sonar/filtered`.
   - Test: injecter frames synthétiques (bruit connu) → valider SNR sortie.

**5. Package `tracking` — Extraction bords**
   - Souscrit `/sonar/filtered`, détecte bords cage, publie `Borders`.
   - Test: frames avec cage simulée (position connue) → valider détection.

**6. Package `localisation` — Pose relative**
   - Souscrit `Borders`, calcule pose (x,y,heading), publie `PoseRelative`.
   - Test: bords synthétiques → comparer pose calculée vs attendue (tolérance ±5%).

**7. Package `control` — Asservissement PID**
   - Souscrit `PoseRelative` + `State` (mission), calcule commandes, publie `Twist`.
   - Test: injecter poses fixes → valider commandes PID (pas de saturation, convergence).

**8. Package `mission` — Machine d'états**
   - Gère transitions IDLE/LOCK_ON/APPROACH/DOCKING, active/désactive control.
   - Test: simuler scénarios (acquisition réussie, perte tracking, abort opérateur).

**9. Package `affichage` — Visualisation (optionnel mais utile tôt)**
   - Aide au debug: affiche sonar brut/filtré, overlay tracking, pose.
   - Test: vérif affichage sans crash (headless mode pour CI).

**10. Package `bringup` — Orchestration**
   - Launch files + YAML params, démarre pipeline complet ou sous-ensembles.
   - Test: lancer chaque config (mock, hardware, replay rosbag).

## Architecture garantissant modularité

**Interfaces strictes (contrats)**
- Chaque package expose topics/services documentés, ne dépend que de `docking_msgs` et `docking_utils`.
- Pas d'import croisé entre packages métiers (sonar n'importe jamais tracking).

**Tests à 3 niveaux**
1. Unitaires: fonctions pures (utils, conversions, PID) → pytest/gtest.
2. Nœud: publisher mock → nœud → vérif sortie → pytest avec rclpy.
3. Intégration: pipeline partiel (sonar→traitement→tracking) via launch + rosbag → validation métriques.

**Paramétrage YAML central**
- `config/default.yaml`: params par défaut tous packages.
- `config/hardware.yaml`, `config/simulation.yaml`: overrides.
- Chaque nœud charge ses params depuis namespace ROS (`/docking/<package>/...`).

## Par quoi commencer (ordre pratique)

**Semaine 1-2: Fondations**
1. Créer `docking_msgs`: définir Frame, Borders, PoseRelative, State → compiler → tester import.
2. Créer `docking_utils`: conversions polaire/cartésien, filtres de base → tests unitaires.
3. Setup workspace: `colcon build`, `.vscode/tasks.json` (build/test/lint), CI basic (GitHub Actions).

**Semaine 3: Pipeline minimal (end-to-end mock)**
4. `sonar` en mode mock: publie frames synthétiques (cage à position fixe).
5. `traitement` pass-through: recopie `/raw` → `/filtered` (validation pipeline).
6. `tracking` détection simple: trouve max intensité → publie 4 coins fixes.
7. `localisation` trivial: moyenne des bords → pose (0,0,0).
8. `control` debug: affiche pose reçue, publie Twist nul.
9. `bringup` minimal: lance tout en mock → valider flux topics (ros2 topic list/echo).

**Semaine 4+: Implémentation réelle**
10. Améliorer `sonar`: intégrer driver Oculus réel.
11. Améliorer `traitement`: filtres adaptatifs (Wiener, morpho).
12. Améliorer `tracking`: algorithme robuste (template matching, contours).
13. Améliorer `localisation`: géométrie 3D cage, filtrage Kalman.
14. Améliorer `control`: tuning PID, anti-windup, saturation.
15. Implémenter `mission`: machine d'états complète + transitions.

## Checklist package "testable indépendamment"

Pour chaque package, valider:
- [ ] `package.xml` avec dépendances explicites (build, exec, test).
- [ ] `setup.py` ou `CMakeLists.txt` correct (install scripts/libs).
- [ ] Paramètres YAML par défaut dans `config/`.
- [ ] README mini dans package: objectif, I/O, exemple lancement, tests.

## Outils recommandés

**Développement**
- `colcon`: build/test workspace.
- `ros2 bag`: enregistrer/rejouer sessions réelles.
- `rqt_graph`: visualiser topics/nœuds.
- `plotjuggler`: analyser signaux (PID, erreurs).

**Tests**
- `pytest` + `launch_pytest`: tests nœuds ROS.
- `colcon test` + `colcon test-result`: rapport unifié.
- Mocks: `unittest.mock` (Python), fixtures pour injecter données.

**Qualité**
- `ament_lint` (flake8, pep257): style Python.
- `ament_copyright`: vérif headers.
- `pre-commit`: hooks git (format, lint avant commit).

## Risques et mitigations

| Risque | Mitigation |
|--------|-----------|
| Messages changent → refactor massif | Figer `docking_msgs` v1 tôt, versionner |
| Packages couplés (imports croisés) | Code review strict, tests d'import |
| Tests lents (matériel requis) | Mocks systématiques, CI sans hardware |
| PID instable (tuning long) | Simulateur cage + auto-tuning script |
| Perte tracking fréquente | Mode RECOVERY dans mission + logs détaillés |

## Livrables par étape (validations)

- Étape 1-2: `colcon build` OK, tests `docking_msgs` + `docking_utils` passent.
- Étape 3: Pipeline mock end-to-end, `ros2 topic echo` montre flux complet.
- Étape 4+: Chaque amélioration testée en isolation (rosbag réel ou mock avancé).

## Prochaine action concrète (ce que je peux faire maintenant)

Je peux générer:
1. **Squelette `docking_msgs`**: fichiers `.msg` + `CMakeLists.txt` + `package.xml`.
2. **Squelette `docking_utils`**: structure package Python + fonctions stub + tests.
3. **Template `package.xml`** réutilisable pour tous packages.
4. **`bringup/launch/mock_pipeline.launch.py`**: lance sonar mock + traitement + tracking + localisation.
5. **`.vscode/tasks.json`**: tâches build/test/lint pour chaque package.

**Recommandation**: commencez par **1 et 2** (messages + utils), puis **4** (pipeline mock) pour valider l'architecture avant d'implémenter la logique réelle.
