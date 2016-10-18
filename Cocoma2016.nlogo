__includes ["bdi.nls" "communication.nls"]

breed [waypoints waypoint]
breed [envconstructors envconstructor]
breed [convois convoi]
breed [enemies enemy]
breed [drones drone]
breed [rockets rocket]
breed [HQs HQ]
directed-link-breed [path-links path-link]
undirected-link-breed [dummy-links dummy-link]
directed-link-breed [convoi-links convoi-link]

globals [mapAlt solAlt basseAlt hauteAlt ; variables topologiques Z discretise: definit le niveau ou se trouvent toutes les informations de la carte (obstacles base etc.) car en 2D, niveau au sol ou se trouvent les agents, niveau basse altitude et niveau haute altitude
  base-patches base-entry base-central ; precache: definit ou se trouvent les patchs de la base d'atterrissage, le patch d'entree sur la piste d'atterrissage, et le patch ou doivent s'arreter les drones pour se recharger. Permet d'evaluer rapidement la distance et les besoins des drones (quand ils doivent rentrer a la base)
  as-cost as-path ; variables globales pour les chemins AStar: le cout d'un pas sur un patch, et as-path est la liste des plans, un pour chaque convoi leader
  max-fuel max-ammo ; fuel and ammo for drones.
  fuel-dec ; how much fuel will be decremented at each iteration
  mission-completed? mission-failed?
  send-interval ; communication period
  is-movie-recording?
  ]

rockets-own [
  origin
  ]

patches-own [
  obstacle? base? hangar? objectif? bridge?; variables topologiques au niveau mapAlt, permet de definir les patchs praticables et ceux qui sont des obstacles
  as-closed as-heuristic as-prev-pos ; variables temporaires pour calculer les chemins AStar (effaces a chaque calcul de plan)
  ]

convois-own[
  incoming-queue
  beliefs
  intentions
  attacked? ; Vrai si le convoi est attaqué
  energy
  finished? ; Is the goal reached ?
  leader?   ; car leading the convoi convoi
  to-protect? ; Should this car be protected at all cost ?
  genlongpath? ; Should the leader compute a new path (but not shortest) ?
  dead?
  speed maxdir ; maximal speed of a car, and max angle
  last-send-time ; communication historical time-stamp
  ]

enemies-own[
  incoming-queue
  dead?
  speed maxdir ; maximal speed of a car, and max angle
  last-send-time ; communication historical time-stamp
  rayon-vision ; rayon de vision des ennemis
  approach?
  mytarget
  mypath
  ]

drones-own[
  incoming-queue
  dead?
  fuel
  finished?
  speed maxdir
  rayon-vision
]



;***********************
;         SETUP
;***********************

to setup
  ; The setup generates environments until one of them is acceptable (the convoi can accomplish the mission)
  let path-is-possible? false
  while [not path-is-possible?] [
    clear-all
    if not debug and not debug-verbose [no-display] ; disable gui display to speedup processing, the time slider won't influence the setup procedure
    setup-globals
    setup-env
    clear-turtles ; reinit the id of the agents
    setup-convois ;

    ifelse nb-cars <= 0 [
      set path-is-possible? true
    ]
    ; generate a path and check is the convoi can reach its destination. If not, generate a new env
    [
      let start-path (plan-astar ([[patch-at 0 0 (pzcor * -1)] of patch-here] of one-of convois with [leader?]) (one-of patches with [objectif?]) false)
      set as-path replace-item 0 as-path start-path
      if not empty? start-path [ set path-is-possible? true]
    ]
  ]
  if not debug and not debug-verbose [no-display]
  setup-drones
  setup-enemies
  ;setup-citizens
  ;setup-hq

  setup-precache
  display ; reenable gui display
  reset-ticks
end

; Initial parameters
to setup-globals
  set mapAlt 0
  set solAlt 1
  set basseAlt (floor max-pzcor / 3 * 2 - 1)
  set hauteAlt (floor max-pzcor - 1)

  set mission-completed? false
  set mission-failed? false

  set as-cost 1 ; cost to move
  set as-path n-values nb-cars [[]] ; max one path for each car

  set send-interval 10 ; in number of steps

 ; set dist-R-set []

  set is-movie-recording? false
end


; Precaches places en global variables for static components in order to speed-up the processes.
to setup-precache
  set base-patches (patches with [base? and pzcor = mapAlt]) ; precache to speedup things
  set base-entry max-one-of (base-patches with-min [pycor]) [pxcor]
  set base-central min-one-of (base-patches with-min [pxcor]) [pycor]
end


;environment definition
to setup-env
  ask patches [set obstacle? false set base? false set hangar? false set objectif? false set bridge? false]

  ;Balles
  set-default-shape rockets "circle"

  ; Herbe
  ask patches with [pzcor = mapAlt][set pcolor green + (random-float 2) - 1]

  ; Montagnes
  ask patches with [ pycor  < 5 and pycor > 2 and pzcor = mapAlt and pxcor < 5 and pxcor > 2 ][ set pcolor white set obstacle? true]


  ; Rivieres
  if nb-rivers > 0 [
    repeat nb-rivers [
      ; A builder will move and create a river at each step
      create-envconstructors 1 [
        ; random deploy on the left side or on the bottom one
        ifelse random-float 1 <= 0.5 [
          set xcor 0
          set ycor random max-pycor
          set heading 90
        ]
        [
          set ycor 0
          set xcor random max-pxcor
          set heading 0
        ]
        set zcor mapAlt

        ; Tag of the first case
        ask patch-here [set pcolor blue set obstacle? true]

        ; move and mark the patch
        repeat max-pxcor + max-pycor [
          ; Change l'orientation aleatoirement
          rt random 30 - 15
          ; one step
          fd 1
          ; randomly select a bridge or a river
          ask patch-here [
            ; bridge
            ifelse random-float 1 <= 0.1 [
              set pcolor brown
              set bridge? true
            ]
            ; River
            [
              set pcolor blue
              set obstacle? true
            ]
          ]
        ]
        die
      ]
    ]
  ]

  ; Lacs
  if nb-lakes > 0 [ ask n-of nb-lakes patches with [pzcor = mapAlt and pxcor > 7 and pycor > 7] [ask patches with [distance-nowrap myself < 4 and pzcor = mapAlt] [set pcolor blue set obstacle? true]] ]

  ; Objectif
  ask one-of patches with[obstacle? = false and base? = false and hangar? = false and pxcor >= (max-pxcor / 2) and pycor >= (max-pycor / 2) and pzcor = mapAlt][set objectif? true ask patch-at 0 0 2 [set pcolor yellow]]

  ; Hangar (la ou les voitures du convois demarrent)
  ask patches with[pzcor = mapAlt and pxcor >= 5 and pxcor < 7 and pycor >= 0 and pycor < 12][set pcolor 8 set hangar? true set obstacle? false]

  ; Base de decollage et atterrissage pour les drones
  ask patches with[pzcor = mapAlt and pxcor >= 3 and pxcor < 5 and pycor >= 0 and pycor < 12][set pcolor 1 set base? true set hangar? false set obstacle? false] ; piste verticale
  ask patches with[pzcor = mapAlt and pycor = 0 and pxcor >= 0 and pxcor < 18][set pcolor 1 set base? true set hangar? false set obstacle? false] ; piste horizontale

  ; Batiment (pour faire joli, ne sert a rien fonctionnellement)
  ask patches with[pzcor <= solAlt and pxcor >= 0 and pxcor < 3 and pycor >= 0 and pycor < 5][set pcolor 3 set obstacle? true set base? false set hangar? false] ; Batiment
  ask patches with [pzcor < 5 and pxcor = 0 and pycor = 0 and pzcor > 0 ] [ set pcolor 3 set obstacle? true set base? false set hangar? false] ; Antenne

  ; Copie des obstacles: on s'assure que les patchs au niveau solAlt ont la meme valeur obstacle? que leur patch en-dessous au niveau mapAlt (assure que enemy-random-move fonctionne bien et facilite la detection des obstacles car pas besoin de regarder au niveau mapAlt mais directement dans les patchs solAlt)
  ask patches with [[obstacle?] of patch-at 0 0 -1] [set obstacle? true]
end

to setup-enemies
  if nb-enemies > 0 [
    create-enemies nb-enemies
    ask enemies [
      set shape "car"
      set color red

     ; Init vars enemies
      set speed 0.05 * simu-speed
      set maxdir 10 * simu-speed
      set roll 0
      set pitch 0
      set approach? false
      set dead? false
      set mytarget nobody
      set mypath []
      set rayon-vision vision-enemy
      set heading one-of [ 0 90 180 270 ] ; Dans une direction aléatoire
    ]

    ask turtle-set enemies [
      let rand one-of (patches with [not hangar? and pzcor = mapAlt and not obstacle? and not base? and not any? turtles-here])
      setxy [pxcor] of rand [pycor] of rand ; les ennemis n'apparaissent pas sur un obstacle
      set zcor solAlt

    ]
  ]
end

to setup-drones

if nb-drones > 0 [
  create-drones nb-drones
  ask drones [
    set shape "airplane"
    set color cyan

    set speed 0.05 * simu-speed
    set maxdir 10 * simu-speed
    set finished? false
    set fuel 60
    set roll 0
    set pitch 0
    set rayon-vision vision-drone
    set heading random 360

  ]
    ask turtle-set drones [
      let rand one-of (patches with [pzcor > 0 and pzcor <= hauteAlt])
      setxyz [pxcor] of rand [pycor] of rand [pzcor] of rand
    ]


]
end


to setup-convois
  if nb-cars > 0 [
    ; get the size of the base to deploy the car accordingly
    let base-min-pxcor min [pxcor] of (patches with [hangar? and pzcor = mapAlt])
    let base-max-pxcor max [pxcor] of (patches with [hangar? and pzcor = mapAlt])
    let base-min-pycor min [pycor] of (patches with [hangar? and pzcor = mapAlt])
    let base-max-pycor max [pycor] of (patches with [hangar? and pzcor = mapAlt])

    ; creation des voitures du convoi et cortege
    create-convois nb-cars
    ask convois
    [
      ; Init apparence NetLogo
      set shape "car"
      set color magenta

      ; Init des structures BDI
      set incoming-queue [] ; Do not change

      ; Init vars convois
      set speed 0.05 * simu-speed
      set maxdir 10 * simu-speed
      set heading 0
      set roll 0
      set pitch 0
      set finished? false
      set leader? false
      set to-protect? false
      set genlongpath? false
      set dead? false
      set energy convoi-energy

      ; Visu
      set label who ; display the car names
    ]

    ; get the id of the first one
    let first-car min [who] of convois
    let last-car max [who] of convois

    ; configure the leader
    ask convoi first-car [
      set leader? true
      set color orange
      move-to patch base-max-pxcor base-max-pycor 1
    ]

    ; configure the last car as the critical one
    ask convoi last-car [
      set to-protect? true
      set color yellow
    ]

    ; deploying the other car
    if nb-cars > 1 [
      ; ask non leader cars
      ask turtle-set sort-on [who] convois with [who > first-car]
      [
        ; we create a link between them
        create-convoi-link-to turtle (who - 1)
        ;if who >= 4 and who mod 2 = 0 [ create-convoi-link-with turtle (who - 3) ]

        ; deploying
        ifelse (who - 1) mod 2 = 0 [ set xcor base-min-pxcor ] [ set xcor base-max-pxcor ] ; a gauche ou a droite selon le nombre (pair ou impair respectivement)
        set ycor base-max-pycor - (floor (who / 2) / (nb-cars / 2) * (base-max-pycor - base-min-pycor)) ; d'une rangee de plus en plus basse toutes les deux voitures
        set zcor solAlt
      ]

    ]
  ]
end


;------------------------------------------------------------
;------------- functions ------------------------------------
;------------------------------------------------------------

; Plannification AStar d'un patch start vers un patch goal
; Note: si l'heuristique est consistante/monotone (comme distance euclidienne/vol d'oiseau), h = 0 revient a faire Djikstra
; Note2: on l'utilise avec le convoi mais on peut l'utiliser avec n'importe quel agent, c'est generique.
; Note3: limite en 2D pour cette application mais on peut facilement la modifier pour accepter la 3D (enlever les limites with [pzcor ...])
to-report plan-astar [start goal longpath?] ; start et goal sont des patchs

  ; Desactivation du refresh GUI (car calculs internes): Pour etre plus rapide, on dit a NetLogo qu'il peut calculer toute cette fonction sans avoir a updater le GUI (que des calculs internes), comme ca le slider de vitesse n'influencera pas la vitesse de ce code (sinon en slower ca met vraiment beaucoup de temps)
  if not debug-verbose [no-display]

  ; INIT
  ; Ajustement du niveau du but par rapport au start, car le plan est en 2D ici
  let start-pzcor [pzcor] of start
  set goal [patch-at 0 0 ([pzcor] of start - [pzcor] of goal)] of goal

  ; (Re)init des variables AStar sur tous les patchs
  ;let closed n-values world-height [n-values world-width [0]]
  ask patches [
    set as-closed 0 ; sert a savoir si ce patch a deja ete visite. 0 = non visite, 1 = deja visite (et on visite en premier par le chemin optimal comme Djikstra, donc si un noeud a deja ete visite, on est sur qu'il est inutile de le revisiter par un autre chemin puisqu'il sera moins optimal que le premier chemin qui a conduit a ce patch - ceci est assure car on utilise la distance euclidienne a vol d'oiseau qui est une heuristique consistante/monotone, pas juste admissible)
    set as-heuristic astar-faster * distance-nowrap goal ; si astar-faster > 1 alors on utilise Weighted AStar, ou le chemin est suboptimal avec une limite de cout au plus astar-faster fois supérieur au cout du chemin optimal. (eg: astar-faster = 2 signifie que le chemin sera au pire deux fois moins optimal au pire). Note: si astar-faster = 0 alors h = 0 pour tous les patchs et ca revient à l'algo de Dijkstra.
  ]

  ; Init de l'algo en utilisant le patch de depart
  let pos start
  let h [as-heuristic] of start
  let g 0
  let f (g + h)

  ; Init de la liste open (la liste des patchs a explorer) du type [f, g, h, position du patch]
  let open (list (list f g h pos))

  ; Init des criteres d'arret
  let found false ; si un chemin a ete trouve
  let resign false ; si aucun chemin ne peut etre trouve (plus rien dans la liste open)
  let counter 0 ; si on a visite trop de patchs et que la recherche met trop de temps

  while [not found and not resign] [
    ; Critere d'arret si echec (plus de patch a visiter ou trop de patchs deja visite)
    ifelse empty? open or (astar-max-depth > 0 and counter > astar-max-depth) [
      set resign true
    ]
    [
      ; Incremente le counter
      set counter counter + 1

      ; On reorganise la liste open pour toujours visiter le meilleur patch candidat en premier (celui qui maximise f)
      set open sort-by [item 0 ?1 < item 0 ?2] open
      ; Cas particulier: on visite le plus mauvais patch, celui qui minimise f, pour maximiser la longueur du chemin (cool pour tester les drones car l'environnement reste relativement petit)
      if astar-longpath or longpath? [set open reverse open]
      ; Autre cas particulier: on visite le chemin au hasard, permet aussi de construire un long chemin (mais moins long) et plus rapidement. C'est un compromis entre l'optimal et la longueur.
      if astar-randpath [set open shuffle open]

      ; Pop un element de la liste, le meilleur candidat
      let next first open
      set open but-first open
      set pos item 3 next
      set g item 1 next

      ; Dessin en live du chemin parcouru par astar
      if debug-verbose [
      wait 0.01
      ask pos [ set pcolor red ]
      ]

      ; Critere d'arret si reussite: on est sur le but donc on a trouve un chemin
      ifelse pos = goal [
        set found true
      ]
      ; Sinon on va explorer les voisins du patch en cours
      [
        ; Expansion du meilleur candidat (expansion = on ajoute les voisins dans la liste open, des noeuds a visiter)
        ask [neighbors6-nowrap with [pzcor = start-pzcor and as-closed = 0 and not obstacle? and not base?]] of pos [ ; On ne visite que les voisins au meme niveau (astar en 2D, mais on peut etendre ici au 3D facilement!) ET on ne l'a pas deja visite (as-closed = 0) ET il n'y a pas d'obstacle sur ce patch
          ; Calcul du score f de ce voisin
          let g2 g + as-cost
          let h2 as-heuristic
          let f2 g2 + h2

          ; Ajout dans la liste open des patchs a visiter
          set open lput (list f2 g2 h2 self) open

          ; Ajout des meta-donnees sur ce patch
          ;set as-closed min (list ((as-closed + 1) ([as-closed] of pos + 1)) ; Pas necessaire car on est sur qu'on ne visite qu'une fois un noeud dans open, ensuite on lui attribue un nombre dans closed et donc on ne l'ouvrira plus jamais
          set as-closed ([as-closed] of pos + 1) ; pour savoir que ce patch a deja ete visite + faire astar-visu-more
          set as-prev-pos pos ; pour backtracker ensuite et trouver le chemin qui mene au but
        ]
      ]
    ]
  ]

  if debug [print (word "found:" found " - resign:" resign)]

  ; Visualisation de tous les noeuds explores en coloriant selon quand ca a ete explore (score as-closed)
  if astar-visu-more [
    let max-closed max [as-closed] of patches with [pzcor = start-pzcor] ; Récupère la valeur tdval max entre tous les patchs
    let min-closed min [as-closed] of patches with [pzcor = start-pzcor] ; Idem pour min tdval
    if (max-closed != min-closed) [ ; Si on a au moins appris quelquechose (sinon tous les patchs auront la même couleur, ce n'est pas intéressant)
      ask patches with [pzcor = start-pzcor] [
        if debug [set plabel precision as-closed 1]
        set pcolor (61 + ((as-closed - min-closed) / (max-closed - min-closed)) * 9 )
      ]
    ]
  ]

  ; Extraction du chemin par marche inverse, depuis le goal vers start (grace a as-prev-pos qui memorise depuis quel patch on est arrive a celui en cours, et donc le chemin le plus court puisque l'algo garantie que la premiere exploration est toujours optimale)
  let path []
  if not resign [
    ; On commence du but, goal
    set pos goal
    set path lput pos path

    ; Pour la visualisation du chemin, init du premier waypoint
    if astar-visu [
      if any? waypoints [
        ask waypoints [ die ]
      ]
      create-waypoints 1 [ hide-turtle move-to [patch-at 0 0 1] of goal ]
    ]

    ; Tant qu'on a pas reconstruit tout le chemin vers le debut, start
    ; On va a chaque fois recuperer le noeud parent avec as-prev-pos
    while [pos != start] [

      ; Visualisation du chemin, on ajoute un lien entre le parent et le noeud en cours
      if astar-visu [
        create-waypoints 1 [ hide-turtle move-to [patch-at 0 0 1] of ([as-prev-pos] of pos)
          create-path-link-to one-of waypoints-on [patch-at 0 0 1] of pos [
            set color red
            show-link
          ]
        ]
      ]

      ; Construction inverse du chemin, on ajoute le noeud parent dans le chemin et on va l'explorer
      ;set pos [min-one-of neighbors6-nowrap [as-closed]] of pos
      set pos [as-prev-pos] of pos
      set path lput pos path
    ]

    ; Chemin construit, on inverse la liste pour qu'elle soit de start a goal au lieu de l'inverse
    set path reverse path
    set path but-first path ; on enleve le premier patch, qui est celui sur lequel on est deja
  ]

  ; Reactivation du refresh GUI
  display

  ; Et on retourne le chemin complet (ou une liste vide si on n'a rien trouve)
  report path
end


; Return the 6 neighbours without the world wrap
to-report neighbors6-nowrap
; reports neighbors-nowrap-n or the indicated size
report neighbors6 with
[ abs (pxcor - [pxcor] of myself) <= 1
  and abs (pycor - [pycor] of myself) <= 1
]
end


;-----------
;  CONVOIS
;-----------

; Procedure principale de gestion des convois
to convois-think

  if nb-cars > 0 [

    let first-car min [who] of convois

    ; Calcul du plan AStar pour chaque leader si necessaire
    foreach sort-on [who] turtle-set convois with [leader? and not finished? and not dead?] [
      let id ([who] of ?) - first-car
      ; Recalcule le chemin si nécessaire (par exemple au début de la simulation ou quand le convoi se sépare)
      ; Note: on est oblige de le faire en dehors du ask sinon on ne peut pas acceder a tous les patchs
      if empty? as-path or length as-path < (id + 1) or empty? (item id as-path) [ ; s'il n'y a pas encore de chemin du tout, ou pas de chemin pour cette voiture, on cree un plan AStar
        ; Cree le plan AStar (attention a ca que le patch start soit au niveau ou il y a les obstacles, ici pzcor = mapAlt pour les obstacles)
        let start-patch min-one-of (patches with [pzcor = mapAlt and not obstacle?]) [distance ?] ; on s'assure de choisir comme patch de depart un patch libre sans obstacle, sinon quand on split un convoi il se peut qu'il soit sur un obstacle et qu'il ne puisse jamais generer de chemin
        let new-path plan-astar ([patch-at 0 0 (pzcor * -1)] of start-patch) (one-of patches with [objectif?]) ([genlongpath?] of ?)
        ; S'il n'y a pas de plan et qu'on a essayé de trouver un long chemin, on attend la prochaine iteration et on reessaie mais avec un plan court
        if empty? new-path and [genlongpath?] of ? [ ask ? [ set genlongpath? false ] ]
        ; S'il n'y a pas deja une entree pour cette voiture on la cree
        ifelse length as-path < (id + 1) [
          set as-path lput new-path as-path
        ]
        ; Sinon on remplace l'entree pour cette voiture par le nouveau plan
        [
          set as-path replace-item id as-path new-path
        ]
      ]
    ]

    ; Deplacement des leaders sur le chemin AStar
    ask convois with [leader? and not finished? and not dead?] [ ; Tant qu'on n'a pas atteint le but
      ;move-convoi-naive ; deplacement naif sans AStar

      ; Recupere le plan AStar
      let my-as-path item (who - first-car) as-path
      if not empty? my-as-path [
        ; Deplacement par waypoints: on se deplace jusqu'au prochain patch du chemin jusqu'à l'atteindre
        let next-patch first my-as-path
        let zz pzcor
        set next-patch [patch-at 0 0 (zz - pzcor)] of next-patch ; mise a niveau de pzcor au cas ou le chemin a ete calculé sur un autre plan
        ; Deplacement vers le prochain waypoint
        if next-patch != patch-here [move-convoi next-patch false false]
        ; Si on a atteint ce patch, on le supprime de la liste, et on va donc continuer vers le prochain patch du chemin
        if patch-here = next-patch [
          set my-as-path remove-item 0 my-as-path
          set as-path replace-item (who - first-car) as-path my-as-path
          if debug [ show (word "Waypoint atteint: " patch-here ", prochain: " next-patch ) ]
        ]
      ]

      ; Critere d'arret: on est a cote de l'objectif
      check-convoi-finished

    ]

    ; Deplacement des voitures-cortege: elles ne font que suivre la voiture devant eux (avec laquelle elles sont liées)
    ask convois with [not leader? and not finished? and not dead?] [
      ifelse any? my-out-convoi-links [
        move-convoi ([patch-here] of one-of out-convoi-link-neighbors) true true
      ]
      ; S'il n'y a pas de lien devant, c'est probablement que la voiture est morte, donc on devient leader
      [
        set leader? true
        set genlongpath? true
        if not to-protect? [ set color orange ]
      ]
    ]
  ]
end

; Supprime le lien entre deux voitures du convoi
; Le voisin de la voiture à protèger ne peut pas s'en séparer !
to separate [turtle1 turtle2] ; Ici, turtle1 précède turtle2
  let last-car max [who] of convois
  if [who] of turtle1 = last-car [ show (word "Erreur : " turtle1 " est à protéger !") stop ]
  ; Si turtle1 n'a pas de suivant
  if not any? [out-convoi-link-neighbors] of turtle1 [ show (word "Erreur : " turtle1 " n'a pas de suivant ") stop ]
  let suivant one-of [out-convoi-link-neighbors] of turtle1
  ; Si le suivant de turtle1 n'est pas turtle2
  if [who] of suivant != [who] of turtle2 [ show [who] of suivant show [who] of turtle2 show (word turtle1  " et "  turtle2  " ne sont pas reliés !") stop ]

  ask turtle1 [
    ; Suppression du lien
    ask convoi-link [who] of turtle1 [who] of turtle2 [ die ]

    ; turtle1 devient leader
    set leader? true
    set genlongpath? true
    if not to-protect? [ set color orange ]
  ]
end

; Renvoie vrai si la voiture à protéger appartient au même convoi que turtle1
; Utile car lors de la séparation (cas d'attaque), le convoi qui a la voiture à protéger doit faire un détour
to-report convoi-to-protect [turtle1]
  if [to-protect?] of turtle1 [ report true ]
   ; S'il n'a pas de voiture derrière lui, faux car la voiture à protéger est la dernière
    if not any? [in-convoi-link-neighbors] of turtle1 [ report false ]
    ; Récursion
    let suivant one-of [in-convoi-link-neighbors] of turtle1
    show suivant
    report convoi-to-protect suivant
end


; turtle1 demande à turtle2, le leader du convoi visé de se reformer
to aggregate [turtle1 turtle2]
  ;Verifier si turtle2 est bien un leader et qu'il est dans le champ de communication

  ;



  ; Verifier si turtle1 n'a pas de suivant - si turtle2 n'a pas de précédent
  if not any? [out-convoi-link-neighbors] of turtle1 and not any? [in-convoi-link-neighbors] of turtle2 [
    ask turtle1 [
      create-convoi-link-to turtle2 ; Création du lien
    ; turtle1 n'est plus leader et reprend sa couleur initiale
      set leader? false
      set genlongpath? false
      if not to-protect? [ set color magenta ]
    ]
  ]

end


to-report detect-obstacle
 if any? other patches in-cone-nowrap 10 60 with [obstacle?] [report true]
; if any? other patches in-cone-nowrap 10 90 [report true]
; if any? other patches in-cone-nowrap 3 270 [report true]
 report false
end

to turn-away
   ;let free-patches neighbors with [not any? patches ]
   ;if any? free-patches [face one-of free-patches]
   rt random 10 - 5
end

to check-convoi-finished
  ; Critere d'arret: on est a cote de l'objectif
  ; Note: on veut etre a cote de l'objectif et pas directement dessus car on est une voiture, donc il se peut qu'on tourne indefiniment autour sans arriver directement a arriver dessus a cause de la limite d'angle de rotation.
  if any? [neighbors6-nowrap with [objectif?]] of patch-here [ ; On ne bouge pas si on est arrive au but!
                                                               ; Fini pour le leader
    set finished? true
    ; Fini aussi pour toutes les voitures-cortege qui suivent ce leader
    let linked-cars (list in-convoi-link-neighbors)
    while [not empty? linked-cars] [ ; on fait une boucle pour recursivement mettre a finished? = true toutes les voitures liees entre elles dans ce cortege
      let next-linked-cars []
      foreach linked-cars [
        ask ? [
          set finished? true
          if any? in-convoi-link-neighbors [ ; on recupere les voitures-cortege liees a la voiture-cortege en cours
            set next-linked-cars lput in-convoi-link-neighbors next-linked-cars
          ]
        ]
      ]
      set linked-cars next-linked-cars
    ]
  ]
end

; Avancer une voiture
; Permet de faire avancer les voitures d'un convoi (cortege et leader)
; Maintien egalement une petite distance afin de ne pas "rentrer" dans la voiture de devant
to move-convoi [goal slowdown? cortege?]
  ;show (word "ici:" patch-here " goal:" goal)

  ; Calcul de l'angle avec la cible
  let headingFlag heading
  ifelse cortege?
  [ set headingFlag (towards goal) ] ; Si c'est un cortege, on veut qu'il suive toujours le leader par le chemin le plus court (surtout en play-mode ou le joueur n'est pas limite par le nowrap)
  [ set headingFlag (towards-nowrap goal) ]
  let dirCorrection subtract-headings headingFlag heading
  ; Arrondissement de l'angle (on ne veut pas faire de micro tournant)
  set dirCorrection precision dirCorrection 2
  ; Limite de l'angle, pour que ce soit plus realiste (la voiture ne peut pas faire un demi-tour sur place!)
  ifelse dirCorrection > maxdir [ ; limite a droite
    set dirCorrection maxdir
  ]
  [
    if dirCorrection < maxdir * -1 [ ; limite a gauche
      set dirCorrection maxdir * -1
    ]
  ]

  ; On tourne
  rt dirCorrection

  ; Limite de vitesse pour les voitures-cortege (pour pas qu'elles ne rentrent dans la voiture leader)
  let tmp-speed speed
  if slowdown? [
    if distance-nowrap goal < 1.1 [
      set tmp-speed tmp-speed / 20
    ]
    if distance-nowrap goal < 0.9 [
      set tmp-speed 0
    ]
  ]

  ; Deplacement!
  set pitch 0 ; make sure there's no pitch ever, else the car will disappear in the ground
  fd tmp-speed ; Avance
end

to convoi-touched [target]
  let msg "no_message"
  ask target [
    ; Perte d'energie et mort éventuelle
    set energy energy - 10
    ;show ("Aie")
    if energy <= 0 [
      set dead? true
      set nb-cars nb-cars - 1
      die
    ]

    if any? out-convoi-link-neighbors [ ; out pointe vers la voiture suivante
      let suivant out-convoi-link-neighbors
      ; Tentative de communication avec le suivant
      convoi-communicate "inform" suivant "separation"
      ask suivant [
        show incoming-queue
        ; Separation si bien recu
        set msg get-message
        if msg != "no_message" and get-content(msg) = "separation" [
          separate target self
        ]
      ]
    ]
    if any? in-convoi-link-neighbors [ ; in pointe vers la voiture précédente
      let precedent in-convoi-link-neighbors
      ; Tentative de communication avec le précédent
      convoi-communicate "inform" precedent "separation"
      ask precedent [
        show incoming-queue
        ; Separation si bien recu
        set msg get-message
        if msg != "no_message" and get-content(msg) = "separation" [
          separate self target
        ]
      ]
    ]
  ]
end

; Utilisé dans les cas de communication voiture-voiture d'un même convoi / voiture - leader d'un autre convoi / voiture - leader drone / leader convoi - leader drone
to convoi-communicate [perf receiver content]
  ; Verifie si le receiver est à portée
  if any? receiver in-radius-nowrap convoi-communication [
    let m create-message perf
    set m add-receiver first [who] of receiver m
    set m add-content content m
    send m
  ]
end


;-----------
;  ENEMIES
;-----------

;Case nord par rapport à l'agent
to-report nord
  report patch-right-and-ahead 0 1
end
;Case nord-est par rapport à l'agent
to-report nord-est
  report patch-right-and-ahead 45 1
end
;Case est par rapport à l'agent
to-report est
  report patch-right-and-ahead 90 1
end
;Case nord-ouest par rapport à l'agent
to-report nord-ouest
  report patch-right-and-ahead -45 1
end
;Case ouest par rapport à l'agent
to-report ouest
  report patch-right-and-ahead -90 1
end

to-report sud
  report patch-right-and-ahead 180 1
end


to move-enemy
  ask enemies [
    if not approach? or mytarget = nobody [
      ; Mouvement aléatoire
      if random-float 1 <= 0.08 [ ;Probabilité de changer de direction
        let pos one-of neighbors4 with
        [ abs (pxcor - [pxcor] of myself) <= 1
          and abs (pycor - [pycor] of myself) <= 1
          and not obstacle?
          and not any? other turtles-here
        ]
        if pos != nobody [
          face pos ; Je me tourne vers une case voisine envisageable
        ]

      ]

      ;let under patch-ahead 1
      ;set under one-of patches with [pxcor = [pxcor] of under and pycor = [pycor] of under and pzcor = mapAlt] ; La case est au niveau mapAlt ! Test sur la case devant l'agent, au niveau d'en dessous

      if (not [obstacle?] of patch-ahead 1 ; Ne roule pas dans l'eau/Sur la montagne
        and not any? other turtles-on patch-ahead 1 ; Ne va pas sur la même case qu'un autre agent
        ; Pas d'effets de bord
        and abs ([pxcor] of patch-ahead 1 - [pxcor] of self) <= 1
        and abs ([pycor] of patch-ahead 1 - [pycor] of self) <= 1
        )[fd speed] ; Avance
    ] observe-enemy  ; regarde autour de lui dans un rayon de rayon-vision
  ]

  foreach sort-on [who] turtle-set enemies with [approach?]
  [ approach ?]



end

to observe-enemy ; Agit en fonction de l'agent présent dans son champ de vision

    if any? convois in-cone-nowrap rayon-vision 180 [

      ;show (word "Je vois " count convois in-radius-nowrap rayon-vision " voiture(s) du convoi")
      let target min-one-of convois in-cone-nowrap rayon-vision 180 [distance-nowrap myself] ; cible la plus proche
      let l [patches in-radius-nowrap portee-enemy] of target
      set l l with [not obstacle?]
      set mytarget min-one-of l [distance-nowrap myself]
      ; Verifie si sa cible est atteignable
      ifelse (distance-nowrap target <= portee-enemy)
      [set heading towards-nowrap target attack-enemy(target)]
      [ set approach? true ]
    ]

  if any? drones in-radius-nowrap rayon-vision [

    let target min-one-of drones in-radius-nowrap rayon-vision [distance-nowrap myself] ; cible la plus proche
      ask target [set color yellow]
      ; Verifie si sa cible est atteignable
      if (distance-nowrap target <= portee-enemy)  [attack-enemy(target)]

    ]
  ;Après - if any? enemies in-radius-nowrap rayon-vision [ communiquer ]
end

to attack-enemy [target] ; L'ennemi tire sur la cible
  let touch? false
  if (ticks mod 10 = 0) [
    hatch-rockets 1 [ ; Création de la balle
      set size 0.3
      setxyz xcor ycor zcor ; Part de l'ennemi
      set color red
      ; Probabilité de rater son tir
      let angle towards-nowrap target ; angle pour se tourner vers la cible
      if (random-float 1.0 >= awkwardness) [ set angle angle + (random 41 ) - 20 ]
      set heading angle
      set origin myself
    ]
      ifelse [breed] of target = convois [
        ask rockets [
          ; Continue d'avancer tant que la cible n'est pas atteinte et dans la portée de l'agent
          while [ any? (turtle-set origin) in-radius-nowrap portee-enemy and not any? convois-on patch-here][ fd 0.5 ]
          ifelse any? convois-on patch-here [
            ask rockets-on convois [ set touch? true set shape "fire" set size 1 wait 0.5 die ] ; Si la balle atteint la cible
          ]
          [ set shape "fire" set size 1 wait 0.3 die ] ; Disparait sinon
        ]
        if (touch?) [
          convoi-touched target
        ]
      ] [ ask rockets [
        face-nowrap target
        ; Continue d'avancer tant que la cible n'est pas atteinte et dans la portée de l'agent
        while [ any? (turtle-set origin) in-radius-nowrap portee-enemy and not any? drones-on patch-here][ fd 0.5 ]
        ifelse any? drones-on patch-here [
          ask rockets-on drones [ set touch? true set shape "fire" set size 1.2 wait 0.5 die ] ; Si la balle atteint la cible
        ]
        [ set shape "fire" set size 1 wait 0.3 die ] ; Disparait sinon
      ]
      if (touch?) [
         drone-touched target
      ]
      ]
  ]

end

; Cas où la cible est hors de portée d'un ennemi - il se rapproche
to approach [ennemi]
  ;let path nobody
  ;if [mypath] of ennemi = [] [
  let path plan-astar ([patch-here] of ennemi) ([mytarget] of ennemi) false;]
  ;calcul du plus court chemin vers la cible ?
  ; Si pas de chemin, l'ennemi abandonne
  if path = [] [ask ennemi [set approach? false set mypath [] set mytarget nobody] stop]
  ; S'il y a un chemin et que l'ennemi n'en a pas, il le suit
  ask ennemi [
    if mypath = [] [set mypath path]
   ; foreach mypath [ask ? [set pcolor pink]]
    let next first mypath
    if next != patch-here [

      set heading towards-nowrap next
      set pitch 0
      if not any? other turtles-on patch-ahead 1 [fd speed]
    ]
    if patch-here = next [ set mypath remove-item 0 mypath ]
  ]
end


;-----------
;  DRONES
;-----------

to move-drone ;Aleatoire pour l'instant
 ask drones [
   if random-float 1 <= 0.4 [ ;Probabilité de changer de direction
      ; Choix de la case suivante (case adjacente/inoccupée)
      let pos one-of neighbors4 with
      [ abs (pxcor - [pxcor] of myself) <= 1
        and abs (pycor - [pycor] of myself) <= 1
        and abs (pzcor - [pzcor] of myself) <= 1
        and not obstacle? ;Evite le lac
        and not any? other turtles-here ;Ne va pas sur la même case qu'un autre drone
      ]

      if pos != nobody [ ; Je me tourne vers une case voisine
        face pos
      ]
      if (not [obstacle?] of patch-ahead 1)[fd speed] ; J'avance
      ;observe-drone ; regarde autour de lui dans un rayon de rayon-vision
    ]
 ]
end

to observe-drone ; Agit en fonction de l'agent présent dans son champ de vision
  if any? enemies in-radius-nowrap rayon-vision [
    let target min-one-of enemies in-radius-nowrap rayon-vision [distance-nowrap myself] ; cible le plus proche
    set heading towards target ; Se tourne vers la cible
    attack-drone(target)
  ]
end

to attack-drone [target] ; Le drone tire sur la cible
  let touch? false
  if (count enemies in-radius-nowrap rayon-vision) > 0 [
    hatch-rockets 1 [ ; Création de la balle
      set size 0.2
      setxyz xcor ycor zcor ; Part du drone
      set color red

      set heading towards target ; Se dirige vers l'ennemie
      set origin myself
    ]
    ask rockets [
      ; Continue d'avancer tant que la cible n'est pas atteinte et dans la portée de l'agent
      while [ any? (turtle-set origin) in-radius-nowrap portee-enemy and not any? enemies-on patch-here][ fd 0.5 ]
     ifelse any? enemies-on patch-here [
       ask rockets-on enemies [ set touch? true set shape "fire" set size 1 wait 1 die ] ; Si la balle atteint la cible
     ]
     [ die ] ; Disparait sinon
    ]
    if (touch?) [ask target [ set dead? true set nb-enemies nb-enemies - 1 die ] ]
  ]
end

to drone-touched [target]
  ask target [
    ; Perte d'energie et mort éventuelle
      set dead? true
      die
    ]
end



to go
  ; Verifie si la portée des ennemis <= leur vision
  if (vision-enemy < portee-enemy) [
    user-message (word "Les ennemis doivent avoir une vision supérieure à leur portée !")
    stop
  ]
  convois-think
  move-enemy
  move-drone
  tick
end
@#$#@#$#@
GRAPHICS-WINDOW
0
0
245
241
-1
-1
10.0
1
10
1
1
1
0
1
1
1
0
20
0
20
0
10
1
0
1
ticks
30.0

SWITCH
15
190
118
223
debug
debug
1
1
-1000

SWITCH
13
235
178
268
debug-verbose
debug-verbose
1
1
-1000

TEXTBOX
13
22
163
40
Environnement \n
12
0.0
1

INPUTBOX
20
55
70
115
nb-cars
2
1
0
Number

BUTTON
17
385
90
418
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

INPUTBOX
81
55
159
115
nb-mountains
3
1
0
Number

INPUTBOX
162
55
214
115
nb-lakes
1
1
0
Number

INPUTBOX
218
55
275
115
nb-rivers
3
1
0
Number

INPUTBOX
673
340
834
400
astar-faster
20
1
0
Number

INPUTBOX
673
414
834
474
astar-max-depth
5000
1
0
Number

SWITCH
481
338
645
371
astar-longpath
astar-longpath
1
1
-1000

SWITCH
481
382
644
415
astar-randpath
astar-randpath
0
1
-1000

SWITCH
482
467
644
500
astar-visu-more
astar-visu-more
1
1
-1000

SWITCH
482
425
645
458
astar-visu
astar-visu
0
1
-1000

SLIDER
17
332
189
365
simu-speed
simu-speed
0
10
3
1
1
NIL
HORIZONTAL

TEXTBOX
9
156
159
174
Debug
12
0.0
1

TEXTBOX
11
302
161
320
Simulation
12
0.0
1

TEXTBOX
471
309
607
327
A*
12
0.0
1

BUTTON
142
386
205
419
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

INPUTBOX
281
55
352
115
nb-enemies
2
1
0
Number

SLIDER
477
54
649
87
vision-enemy
vision-enemy
1
10
10
1
1
NIL
HORIZONTAL

TEXTBOX
476
25
543
43
Ennemis
12
0.0
1

TEXTBOX
690
23
737
41
Convoi
12
0.0
1

SLIDER
689
56
861
89
convoi-energy
convoi-energy
10
100
30
10
1
NIL
HORIZONTAL

INPUTBOX
360
55
430
115
nb-drones
0
1
0
Number

TEXTBOX
480
215
532
233
Drones
12
0.0
1

SLIDER
478
245
650
278
vision-drone
vision-drone
1
10
10
1
1
NIL
HORIZONTAL

SWITCH
205
183
376
216
show-intentions
show-intentions
1
1
-1000

SWITCH
206
232
380
265
show_messages
show_messages
1
1
-1000

SLIDER
690
101
862
134
vision-convoi
vision-convoi
1
10
5
1
1
NIL
HORIZONTAL

SLIDER
689
145
865
178
convoi-communication
convoi-communication
1
10
2
1
1
NIL
HORIZONTAL

SLIDER
478
99
650
132
portee-enemy
portee-enemy
2
10
3
1
1
NIL
HORIZONTAL

SLIDER
479
143
651
176
awkwardness
awkwardness
0
1
0.5
0.1
1
NIL
HORIZONTAL

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fire
false
0
Polygon -2674135 true false 151 286 134 282 103 282 59 248 40 210 32 157 37 108 68 146 71 109 83 72 111 27 127 55 148 11 167 41 180 112 195 57 217 91 226 126 227 203 256 156 256 201 238 263 213 278 183 281
Polygon -955883 true false 126 284 91 251 85 212 91 168 103 132 118 153 125 181 135 141 151 96 185 161 195 203 193 253 164 286
Polygon -1184463 true false 155 284 172 268 172 243 162 224 148 201 130 233 131 260 135 282

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 3D 5.3
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
