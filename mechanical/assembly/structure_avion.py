
# -*- coding: utf-8 -*-

"""
NavVychop
Assemblage paramétrique du fuselage

Les coordonnées des pièces sont en mm.

Repère global :
    X = longitudinal avant -> arrière
    Y = transversal
    Z = vertical

Architecture :
    1. Géométrie des pièces : fichier de référence PIECES_FILE
       (pieces_fuselage.json, partagé avec decoupe_plaque_fuselage.py),
       chargé dans le dictionnaire PIECES
    2. Configuration : ASSEMBLAGE
    3. Construction automatique : boucle d'assemblage

Copyright Yves MICHELS 2026.
This source describes Open Hardware and is licensed under the CERN-OHL-P v2.
You may redistribute and modify this documentation and make products using it
under the terms of the CERN-OHL-P v2 (https://ohwr.org/cern_ohl_p_v2.txt).
This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
PARTICULAR PURPOSE. Please see the CERN-OHL-P v2 for applicable conditions.

"""

import json

import FreeCAD as App
import Part


# ============================================================
# FICHIERS
# ============================================================

NOSE_FILE = (
    r"C:\Users\miche\Documents\Work\Navychop"
    r"\navychop\mechanical\CAD\pointe_avant.FCStd"
)

# Fichier de référence de la géométrie des pièces
# (partagé avec decoupe_plaque_fuselage.py)

PIECES_FILE = (
    r"C:\Users\miche\Documents\Work\Navychop"
    r"\navychop\mechanical\assembly\pieces_fuselage.json"
)

OUTPUT_FILE = (
    r"C:\Users\miche\Documents\Work\Navychop"
    r"\navychop\mechanical\assembly\fuselage_assembly_test.FCStd"
)


# ============================================================
# PARAMETRES GENERAUX
# ============================================================

PLATE_THICKNESS = 3.0
POSITION_AILE_X = 60.0
POSITION_AILE_Z = 12.0
ANGLE_AILE = 2.50


# ============================================================
# POSITION DE LA POINTE AVANT
# ============================================================

NOSE_TRANSLATION = (0.0, 0.0, 0.0)
NOSE_ROTATION = (0.0, 0.0, 0.0)


# ============================================================
# GEOMETRIE DES PIECES
# ============================================================

def load_pieces(path):

    """
    Charge la géométrie des pièces depuis le fichier de référence.

    Retourne {nom: [(x, y), ...]} en mm.
    """

    with open(path, encoding="utf-8") as f:

        data = json.load(f)

    return {
        name: [tuple(point) for point in piece["points"]]
        for name, piece in data["pieces"].items()
    }


PIECES = load_pieces(PIECES_FILE)


# ============================================================
# CONFIGURATION DE L'ASSEMBLAGE
# ============================================================

ASSEMBLAGE = {

    "fuselage_horizontal_avant": {

        "translation": (60.0, -2.5, -3.0),

        "rotation": (0.0, 0.0, 90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },

    "fuselage_horizontal_arriere": {

        "translation": (60.0-300.0, -2.5, -3.0),

        "rotation": (3.4, 0.0, 90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },

    "fuselage_vertical_droit_avant": {

        "translation": (60.0, 0.0, 0.0),

        "rotation": (0.0, -90.0, 90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },

    "fuselage_vertical_gauche_avant": {

        "translation": (60.0, 63.0, 0.0),

        "rotation": (0.0, -90.0, 90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },

    "fuselage_vertical_droit_arriere": {

        "translation": (60.0-300.0, 0.0, 0.0),

        "rotation": (0.0, -90.0, 90.0-1.7),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },

    "fuselage_vertical_gauche_arriere": {

        "translation": (60.0-300.0, 63.0, 0.0),

        "rotation": (0.0, -90.0, 90.0+1.7),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "bord_40x40": {

        "translation": (60.0-650.0+3, 10.0, 20.0),

        "rotation": (0.0, -90.0, 0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "bord_60x60": {

        "translation": (60.0-300.0+3, 0.0, 0.0),

        "rotation": (0.0, -90.0, 0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "queue": {

        "translation": (60-650.0-150, -200.0+30, 60.0),

        "rotation": (0.0, 0.0, 0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "gouverne_profondeur": {

        "translation": (60-650.0-150, -200.0+30, 63.0),

        "rotation": (0.0, 180+10.0, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.50, 0.50),

    },
    
    "aileron_lacet_droit": {

        "translation": (60-650.0-70, 10, 63+80.0),

        "rotation": (0.0, 180-90.0, 180+90.0-1.7),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "aileron_lacet_gauche": {

        "translation": (60-650.0-70, 53, 63+80.0),

        "rotation": (0.0, 180-90.0, 180+90.0+1.7),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "gouverne_lacet_droit": {

        "translation": (60-650.0-70-50, 10, 63+80.0),

        "rotation": (0.0, 180-90.0, 180+90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.50, 0.50),

    },
    
    "gouverne_lacet_gauche": {

        "translation": (60-650.0-70-50, 53, 63+80.0),

        "rotation": (0.0, 180-90.0, 180+90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.50, 0.50),

    },
    
    "box_queue_vertical_droit": {

        "translation": (60-650.0, 7, 60),

        "rotation": (0.0, 180-90.0, 90.0-8),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "box_queue_vertical_gauche": {
 
        "translation": (60-650.0, 50, 60),

        "rotation": (0.0, 180-90.0, 90.0+8),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    
    "box_queue_horizontal": {

        "translation": (60-650.0, 53, 20.0),

        "rotation": (-15, 180, 90.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.90, 0.90),

    },
    

    "tige_carbone_12": {

        "translation": (-POSITION_AILE_X-60, -250+30, 63-POSITION_AILE_Z+12-60*ANGLE_AILE*0.01745329),

        "rotation": (0, 180-ANGLE_AILE, 0.0),

        "thickness": 12,

        "color": (0.50, 0.50, 0.90),

    },

    "plat_aile_droite": {

        "translation": (-POSITION_AILE_X, -3, 63-POSITION_AILE_Z),

        "rotation": (0, 180-ANGLE_AILE, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.50, 0.50, 0.90),

    },

    "avant_aile_droite": {

        "translation": (-POSITION_AILE_X+5, -3, 63-POSITION_AILE_Z+6),

        "rotation": (0, 180-ANGLE_AILE, 0.0),

        "thickness": 2*PLATE_THICKNESS,

        "color": (0.50, 0.50, 0.90),

    },


    "haut_avant_aile_droite": {

        "translation": (-POSITION_AILE_X, -3, 63-POSITION_AILE_Z+9),

        "rotation": (0, 180-ANGLE_AILE+5, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.50, 0.50, 0.90),

    },
    
    "haut_arriere_aile_droite": {

        "translation": (-POSITION_AILE_X-70, -3, 63-POSITION_AILE_Z+12),

        "rotation": (0, 180-ANGLE_AILE-4, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.50, 0.50, 0.90),

    },
    
    
    "aileront_aile_droite": {

        "translation": (-POSITION_AILE_X-258, -3, 63-POSITION_AILE_Z-ANGLE_AILE*240*0.01745329),

        "rotation": (0, 180-ANGLE_AILE-5, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.90, 0.50, 0.50),

    },
    
    "plat_aile_gauche": {

        "translation": (-POSITION_AILE_X, 63, 63-POSITION_AILE_Z),

        "rotation": (0, 180-ANGLE_AILE, 0.0),

        "thickness": PLATE_THICKNESS,

        "color": (0.50, 0.50, 0.90),

    },

    
    "avant_aile_gauche": {
    
        "translation": (-POSITION_AILE_X+5, 63, 63-POSITION_AILE_Z+6),
    
        "rotation": (0, 180-ANGLE_AILE, 0.0),
    
        "thickness": 2*PLATE_THICKNESS,
    
        "color": (0.50, 0.50, 0.90),
    
    },
    
    "haut_avant_aile_gauche": {
    
        "translation": (-POSITION_AILE_X, 63, 63-POSITION_AILE_Z+9),
    
        "rotation": (0, 180-ANGLE_AILE+5, 0.0),
    
        "thickness": PLATE_THICKNESS,
    
        "color": (0.50, 0.50, 0.90),
    
    },
    
    "haut_arriere_aile_gauche": {
    
        "translation": (-POSITION_AILE_X-70, 63, 63-POSITION_AILE_Z+12),
    
        "rotation": (0, 180-ANGLE_AILE-4, 0.0),
    
        "thickness": PLATE_THICKNESS,
    
        "color": (0.50, 0.50, 0.90),
    
    },
    
    "aileront_aile_gauche": {

    "translation": (-POSITION_AILE_X-258, 63, 63-POSITION_AILE_Z-ANGLE_AILE*240*0.01745329),

    "rotation": (0, 180-ANGLE_AILE-5, 0.0),

    "thickness": PLATE_THICKNESS,

    "color": (0.90, 0.50, 0.50),

},

}


# ============================================================
# OUTILS GEOMETRIQUES
# ============================================================

def polygon_to_face(points):

    vertices = [
        App.Vector(float(x), float(y), 0.0)
        for x, y in points
    ]

    vertices.append(vertices[0])

    wire = Part.makePolygon(vertices)

    return Part.Face(wire)


def make_plate(points, thickness):

    face = polygon_to_face(points)

    return face.extrude(
        App.Vector(0, 0, thickness)
    )


def make_placement(translation, rotation):

    rx, ry, rz = rotation

    rot_x = App.Rotation(
        App.Vector(1, 0, 0),
        rx
    )

    rot_y = App.Rotation(
        App.Vector(0, 1, 0),
        ry
    )

    rot_z = App.Rotation(
        App.Vector(0, 0, 1),
        rz
    )

    rotation_finale = (
        rot_z.multiply(
            rot_y.multiply(rot_x)
        )
    )

    return App.Placement(
        App.Vector(*translation),
        rotation_finale
    )


# ============================================================
# RECUPERATION DU SOLIDE FINAL
# ============================================================

def get_final_shape(doc):

    """
    Recherche prioritaire du Tip d'un Body PartDesign.

    Solution de secours :
    récupération du plus gros solide du document.
    """

    for obj in doc.Objects:

        if obj.TypeId == "PartDesign::Body":

            print("Body trouvé :", obj.Name)

            tip = obj.Tip

            if tip is not None:

                print("Tip trouvé :", tip.Name)

                if hasattr(tip, "Shape"):

                    if not tip.Shape.isNull():

                        return tip.Shape.copy()

    # Solution de secours

    candidates = []

    for obj in doc.Objects:

        if not hasattr(obj, "Shape"):
            continue

        if obj.Shape.isNull():
            continue

        if len(obj.Shape.Solids) > 0:

            volume = obj.Shape.Volume

            candidates.append(
                (volume, obj)
            )

    if len(candidates) == 0:

        raise RuntimeError(
            "Impossible de trouver un solide final."
        )

    candidates.sort(
        key=lambda x: x[0],
        reverse=True
    )

    obj = candidates[0][1]

    print(
        "Utilisation du plus gros solide :",
        obj.Name
    )

    return obj.Shape.copy()


# ============================================================
# CREATION GENERIQUE D'UNE PIECE
# ============================================================

def add_plate(
    doc,
    group,
    name,
    points,
    config
):

    """
    Génère une plaque et l'ajoute au groupe FreeCAD.

    Tous les paramètres de placement et de géométrie
    sont fournis par les dictionnaires PIECES et ASSEMBLAGE.
    """

    shape = make_plate(
        points,
        config["thickness"]
    )

    obj = doc.addObject(
        "Part::Feature",
        name
    )

    obj.Label = name.replace("_", " ")

    obj.Shape = shape

    obj.Placement = make_placement(
        config["translation"],
        config["rotation"]
    )

    obj.ViewObject.ShapeColor = config["color"]

    group.addObject(obj)

    return obj


# ============================================================
# VERIFICATION DE LA CONFIGURATION
# ============================================================

def check_configuration():

    missing_geometry = (
        set(ASSEMBLAGE.keys()) - set(PIECES.keys())
    )

    # Le fichier de référence contient aussi des pièces qui ne sont
    # que dans le patron de découpe : elles ne sont pas assemblées ici.

    not_assembled = (
        set(PIECES.keys()) - set(ASSEMBLAGE.keys())
    )

    if missing_geometry:

        raise ValueError(
            "Géométrie manquante pour : "
            + str(missing_geometry)
        )

    print(
        "Configuration valide :",
        len(ASSEMBLAGE),
        "pièces assemblées sur",
        len(PIECES),
        "pièces de référence."
    )

    if not_assembled:

        print(
            "Pièces de référence non assemblées :",
            sorted(not_assembled)
        )


# ============================================================
# CREATION DE L'ASSEMBLAGE
# ============================================================

#Jprint("")
#print("==========================================")
#print("CREATION DE L'ASSEMBLAGE NAVVYCHOP")
#print("==========================================")


# Vérification des données

check_configuration()


# Création du document

assembly = App.newDocument(
    "NavVychop_Assembly"
)


# ============================================================
# POINTE AVANT
# ============================================================

#print("")
#print("Chargement de la pointe avant...")


nose_doc = App.openDocument(
    NOSE_FILE
)

nose_shape = get_final_shape(
    nose_doc
)

#print("Solides récupérés :", len(nose_shape.Solids))
#print("Volume :", nose_shape.Volume, "mm3")


nose_group = assembly.addObject(
    "App::DocumentObjectGroup",
    "Pointe_Avant"
)


nose = assembly.addObject(
    "Part::Feature",
    "PointeAvant"
)

nose.Label = "Pointe avant"

nose.Shape = nose_shape

nose.Placement = make_placement(
    NOSE_TRANSLATION,
    NOSE_ROTATION
)

nose_group.addObject(nose)


# ============================================================
# CREATION ITERATIVE DES PIECES DU FUSELAGE
# ============================================================

#print("")
#print("Création des pièces du fuselage...")


fuselage_group = assembly.addObject(
    "App::DocumentObjectGroup",
    "Fuselage"
)


# Dictionnaire contenant les objets FreeCAD créés

fuselage_objects = {}


for name, config in ASSEMBLAGE.items():

    #print("")
    #print("Création de :", name)

    obj = add_plate(

        doc=assembly,

        group=fuselage_group,

        name=name,

        points=PIECES[name],

        config=config

    )

    fuselage_objects[name] = obj

    #print("  Volume :", obj.Shape.Volume, "mm3")

    #print("  Placement :", obj.Placement)


# ============================================================
# AFFICHAGE
# ============================================================

nose.ViewObject.ShapeColor = (
    0.75, 0.75, 0.75
)

nose.ViewObject.Transparency = 0


# ============================================================
# CALCUL
# ============================================================

assembly.recompute()


# ============================================================
# SAUVEGARDE
# ============================================================

assembly.saveAs(
    OUTPUT_FILE
)


# ============================================================
# FERMETURE DU DOCUMENT SOURCE
# ============================================================

App.closeDocument(
    nose_doc.Name
)


# ============================================================
# BILAN
# ============================================================

#print("")
#print("==========================================")
#print("ASSEMBLAGE CREE")
#print("==========================================")


#print("")
#print("Pointe avant :")
#print("  Translation :", NOSE_TRANSLATION)
#print("  Rotation    :", NOSE_ROTATION)


#print("")
#print("Pièces du fuselage :")

for name, obj in fuselage_objects.items():

    print(
        "  -",
        name,
        "| Volume :",
        round(obj.Shape.Volume, 2),
        "mm3"
    )


print("")
print("Nombre de plaques :", len(fuselage_objects))

print("Epaisseur standard :", PLATE_THICKNESS, "mm")

print("")
print("Fichier :", OUTPUT_FILE)


print("")
print("==========================================")
print("FIN")
print("==========================================")
