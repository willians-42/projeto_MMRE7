/*
 * File:   newmain.c
 * Author: Willians Fernando de Oliveira / Matheus Jose Ferreira Borelli
 * Created on 27 de Julho de 2024, 16:54
 */

//define a frequência em 48MHz
#define _XTAL_FREQ 48000000 

// inclusao de bibliotecas
#include <xc.h>
#include "config_4520.h"
#include "LCD_4520.h"
#include <stdio.h>
#include <string.h>

// Botoes
#define BOTAO_CAFE_CURTO     PORTAbits.RA0
#define BOTAO_CAFE_LONGO     PORTAbits.RA1
#define BOTAO_CAFE_COM_LEITE PORTAbits.RA2
#define BOTAO_CAPPUCCINO     PORTAbits.RA3
#define BOTAO_CHOCOLATE      PORTAbits.RA4
#define BOTAO_CHOCOLEITE     PORTAbits.RA5
#define BOTAO_RESET          PORTAbits.RA6

// Sensor
#define SENSOR_CAPACITIVO PORTCbits.RC0

// Indicadores (LEDs)
#define ENTRADA_AGUA_QUENTE_ESQUERDA       PORTBbits.RB0
#define ENTRADA_AGUA_QUENTE_DIREITA        PORTBbits.RB1
#define ACIONAMENTO_MIXER_ESQUERDA         PORTBbits.RB2
#define ACIONAMENTO_MIXER_DIREITA          PORTBbits.RB3
#define ACIONAMENTO_MOLA_DOSAGEM_CHOCOLATE PORTBbits.RB4
#define ACIONAMENTO_MOLA_DOSAGEM_LEITE     PORTBbits.RB5
#define ACIONAMENTO_MOLA_DOSAGEM_CAFE      PORTBbits.RB6
#define ENERGIZACAO_TERMOBLOCO             PORTBbits.RB7

void setup(void){
    // componentes
    TRISA = 0xFF; // configura todos os pinos de PORTA (botoes) como entrada (0xFF = 11111111)
    PORTA = 0xFF;
    TRISB = 0x00; // configura todos os pinos PORTB (leds indicadores) como saida (0x00 = 00000000)
 
    // display
    TRISD = 0x00; //RD0 a RD7 - saída para o LCD
    PORTD = 0x00; //Coloca portD em 0V.
    lcd_inicia(0x28, 0x0f, 0x06); //lnicializa o display LCD alfanumérico com quatrolinhas de dados.
    
    // sensor capacitivo
    TRISCbits.RC0 = 1;
    SENSOR_CAPACITIVO = 0;
}

// Função para criar um delay em segundos
void delaySegundos(unsigned int segundos) {
    unsigned int i;

    // Configuração do Timer1
    T1CON = 0x00; // Desliga o Timer1 e limpa o registro de controle
    T1CONbits.TMR1CS = 0; // Seleciona o oscilador interno (Fosc/4)
    T1CONbits.T1CKPS = 0b11; // Prescaler de 1:8

    // Calcular o preload para 1 segundo
    // Timer1 overflow ocorre a cada 65536 ciclos de clock
    // Fosc = 48MHz, então Fosc/4 = 12MHz, Prescaler 1:8
    // Timer1 incrementa a cada 0,6667µs (8 / 12MHz)
    // Para 1 segundo (1.000.000µs), precisamos de 1.000.000µs / 0,6667µs ≈ 1.500.000 ciclos de timer
    // Como Timer1 conta até 65536, precisamos de múltiplos ciclos de Timer1 para chegar a 1 segundo
    // Número de ciclos necessários = 1.500.000 / 65536 ≈ 22.875
    // Preload = 65536 - (1.000.000µs / 0,6667µs) % 65536

    // Simplificando o cálculo
    unsigned int overflowCount = 23; // Número de overflows necessários
    unsigned int preload = 65536 - (1500000UL % 65536); // Preload calculado

    for (i = 0; i < segundos; i++) {
        for (unsigned int j = 0; j < overflowCount; j++) {
            TMR1H = (preload >> 8) & 0xFF; // Carregar valor alto do Timer1
            TMR1L = preload & 0xFF; // Carregar valor baixo do Timer1

            PIR1bits.TMR1IF = 0; // Limpa a flag de interrupção do Timer1
            T1CONbits.TMR1ON = 1; // Liga o Timer1

            while (!PIR1bits.TMR1IF); // Espera o overflow do Timer1

            T1CONbits.TMR1ON = 0; // Desliga o Timer1
        }
    }
}

void preAquecimento(void){
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Em aquecimento"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Aguarde..."); //envia String para o Display LCD
    delaySegundos(8);
}

void maquinaPronta(void){
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Coloque o copo e"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("escolha a bebida"); //envia String para o Display LCD
}

void bebidaPronta(void){
    lcd_limpa_tela();
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Bebida pronta"); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Sirva-se!"); //envia String para o Display LCD
    delaySegundos(3);  
}

void selecaoBebidas(int num_bebida){
    char str[25]; // variavel char para armazenar texto
    switch (num_bebida) {
        case 0:
            strcpy(str, "Cafe Curto");
            break;
        case 1:
            strcpy(str, "Cafe Longo");
            break;
        case 2:
            strcpy(str, "Cafe com Leite");
            break;
        case 3:
            strcpy(str, "Cappuccino");
            break;
        case 4:
            strcpy(str, "Chocolate");
            break;
        case 5:  
            strcpy(str, "Chocoleite");
            break;
        default:
            asm("RESET");
            break;  
    }
    lcd_limpa_tela();
    lcd_LD_cursor(0); // inibe a exibicao do cursor
    lcd_posicao (1,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd(str); //envia String para o Display LCD 
    lcd_posicao (2,1);// desloca o cursor para a posicao determinada
    imprime_string_lcd("Preparando..."); //envia String para o Display LCD 
}

void cafeCurto(){
    // Feito - conferir
    PORTB = 0b11001010; // entrada de agua - direita, mixer - direita, molas de dosagem - cafe
    delaySegundos(5);  
    PORTB = 0b10001010; // 5 segundos -  cessa mola de dosagem
    delaySegundos(8); // 
    PORTB = 0b10001000; // 13 segundos - cessa entrada de agua
    delaySegundos(0.25);
    PORTB = 0b10000000; // encerrado
    bebidaPronta();    
}

void cafeLongo(){
    // Feito - Conferir
    PORTB = 0b11001010; // entrada de agua - direita, mixer - direta, molas de dosagem - cafe
    delaySegundos(10); 
    PORTB = 0b10001010; // 10 segundos -  cessa mola de dosagem
    delaySegundos(13); 
    PORTB = 0b10001000; // 23 segundos - cessa entrada de agua
    delaySegundos(0.25); 
    PORTB = 0b10000000; // encerrado
    bebidaPronta(); 
}

void cafeComLeite(){
    // Feito - conferir
    PORTB = 0b11101111; // entrada de agua - direita e esquerda, mixer - esquerda e direita, molas de dosagem - cafe e leite
    delaySegundos(5); 
    PORTB = 0b10100101; // 5 segundos - cessa entrada de agua - direita, mixer - direita e mola de dosagem - cafe
    delaySegundos(10); 
    PORTB = 0b10000101; // 15 segundos - cessa mola de dosagem - leite
    delaySegundos(6); 
    PORTB = 0b10000100; // 21 segundos - cessa entrada de agua -  esquerda
    delaySegundos(0.25);
    PORTB = 0b10000000; // encerrado
    bebidaPronta();   
}

void cappuccino(){
    // Feito - conferir
    PORTB = 0b11111111; // entrada de agua - direita e esquerda, mixer - esquerda e direita, molas de dosagem - chocolate, leite e cafe
    delaySegundos(5); 
    PORTB = 0b10110101; // 5 segundos - cessa entrada de agua - direita, mixer - direita e mola de dosagem - cafe
    delaySegundos(5); 
    PORTB = 0b10010101; // 10 segundos - cessa mola de dosagem - leite
    delaySegundos(5); 
    PORTB = 0b10000101; // 15 segundos - cessa mola de dosagem -  chocolate
    delaySegundos(5);
    PORTB = 0b10000100; // 20 segundos - cessa entrada de agua - esquerda
    delaySegundos(0.25);
    PORTB = 0b10000000; // encerrado
    bebidaPronta();     
}

void chocolate(){
    // Feito - Conferir
    PORTB = 0b10010101; // entrada de agua - esquerda, mixer - esquerda, molas de dosagem - chocolate
    delaySegundos(20); 
    PORTB = 0b10001010; // 20 segundos -  cessa mola de dosagem - chocolate
    delaySegundos(1); 
    PORTB = 0b10001000; // 21 segundos - cessa entrada de agua - esquerda
    delaySegundos(0.25); 
    PORTB = 0b10000000; // encerrado
    bebidaPronta();  
}

void chocoleite(){
    PORTB = 0b10110101; // entrada de agua - esquerda, mixer - esquerda, molas de dosagem - chocolate e leite
    delaySegundos(15); // 
    PORTB = 0b10000101; // 15 segundos - cessa molas de dosagem - chocolate e leite
    delaySegundos(5); // 
    PORTB = 0b10000101; // cessa entrada de agua - esquerda
    delaySegundos(0.25); // 
    PORTB = 0b10000000; // encerrado
    bebidaPronta(); 
}

void prepararBebida(int num_bebida){
    switch (num_bebida) {
        case 0:
            cafeCurto();
            break;
        case 1:
            cafeLongo();
            break;
        case 2:
            cafeComLeite();
            break;
        case 3:
            cappuccino();
            break;
        case 4:
            chocolate();
            break;
        case 5:  
            chocoleite();
            break;
        default:
            break;  
    }
}

void main(void) {
    setup();
    preAquecimento();
    
    int estado = 0;
    int escolha = -1;

    while(1){
        ENERGIZACAO_TERMOBLOCO = 1;
        maquinaPronta();
        if (SENSOR_CAPACITIVO == 1){
            if (BOTAO_CAFE_CURTO == 0 && estado == 0){
                escolha = 0;
                estado = 1;
                __delay_ms(100); // espera um pouco, trata o debounce
            } else if (BOTAO_CAFE_LONGO == 0 && estado == 0) {
                escolha = 1;
                estado = 1;
                __delay_ms(100); 
            } else if (BOTAO_CAFE_COM_LEITE == 0 && estado == 0) {
                escolha = 2;
                estado = 1;
                __delay_ms(100); 
            } else if (BOTAO_CAPPUCCINO == 0 && estado == 0) {
                escolha = 3;
                estado = 1;
                __delay_ms(100); 
            } else if (BOTAO_CHOCOLATE == 0 && estado == 0) {
                escolha = 4;
                estado = 1;
                __delay_ms(100); 
            } else if (BOTAO_CHOCOLEITE == 0 && estado == 0) {
                escolha = 5;
                estado = 1 ;
                __delay_ms(100); 
            } else if (BOTAO_RESET == 0 && estado == 0) {
                escolha = 6;
                estado = 1;
                __delay_ms(100); 
            } else {
                continue;
            }   
            selecaoBebidas(escolha);
            prepararBebida(escolha);
            SENSOR_CAPACITIVO = 0;
            estado = 0;
            escolha = -1;
        } 
    }   
    return;
}
