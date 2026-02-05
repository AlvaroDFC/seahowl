# Developer Documentation

## Classes Overview

This section provides detailed information regarding the User Language classes and their corresponding methods:

* [Main](main-file.md)
* [Turbine](turbine-file.md)
* [Environment](environment-file.md)
* [Blade](blade-file.md)
* [Rna](rna-file.md)
* [Tower](tower-file.md)
* [Floater](floater-file.md)

::: seahowl.core
    options:
      show_root_toc_entry: true

##  Overview

```mermaid
---
config:
  class:
    hideEmptyMembersBox: true
---
classDiagram
    direction LR
    class AAA
    class CCCC
    class CACA
    class ABAB
    class LLLL
    class WWW
    class ADADAD
    class CYCY
    class STST
    class BLC
    class IMPP
    class INFFF
    class CLECLE
    class LEMB
    class LDF
    class FIFI
    class GlobalProperties
    class VVV
    class MGMG

    style AAA fill:#CC565D,stroke-width:4px
    style CCCC fill:#668C5E
    style CYCY fill:#9E8260
    style GlobalProperties fill:#A680AD

    %% --- HÉRITAGE DES COMPOSANTS ---
    VVV <|-- CCCC
    CCCC <|-- CACA
    CCCC <|-- ABAB

    %% --- SYSTÈME BLC ---
    BLC <|-- IMPP
    BLC <|-- INFFF
    BLC <|-- CLECLE

    %% --- MODÈLES DE TRANSPORT ---
    TransportModel <.. LEMB : implements
    TransportModel <.. LDF : implements
    TransportModel <.. FIFI : implements

    %% --- COMPOSITION ET RELATIONS ---
    AAA *-- GlobalProperties : properties
    AAA *-- CYCY : CYCY
    AAA *-- CCCC : CCCCs (list)

    GlobalProperties --> TransportModel

    ABAB *-- LLLL : _LLLLs (list)
    ABAB *-- WWW : _WWW_properties

    LLLL --> ADADAD : ADADAD

    CYCY *-- STST : list
    STST *-- BLC : list_update

    IMPP --> MGMG : uses
    INFFF --> MGMG : uses
```

### Legend

```mermaid
classDiagram
    %% Définition des descriptions
    class L1["Inheritance (Subclass)"]
    class L2["Composition (Attr)"]
    class L3["Association (Ref)"]
    class L4["Implementation (ABC)"]

    %% Liens individuels
    A <|-- L1
    B *-- L2
    C --> L3
    D <.. L4

    %% Styles pour cacher les classes sources et aligner les textes
    style A fill:none,stroke:none,color:none
    style B fill:none,stroke:none,color:none
    style C fill:none,stroke:none,color:none
    style D fill:none,stroke:none,color:none

    style L1 fill:none,stroke:none,font-size:12px
    style L2 fill:none,stroke:none,font-size:12px
    style L3 fill:none,stroke:none,font-size:12px
    style L4 fill:none,stroke:none,font-size:12px
```
